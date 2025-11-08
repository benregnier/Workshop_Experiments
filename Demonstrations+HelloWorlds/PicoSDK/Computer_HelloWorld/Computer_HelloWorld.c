/*
Hello World for Music Thing Modular Workshop Computer

Just the most basic test setup to check the wires are where I expected them to be.

None of this code is recommended or optimised.

Public Domain, do what you will.

Written by Tom Whitwell
In Herne Hill, London
February 2024

tom.whitwell@gmail.com

*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"
#include "pico.h"
#include "hardware/adc.h"
#include <math.h>

// PIN DEFINITIONS

// Mux pins
#define MUX_LOGIC_A 24
#define MUX_LOGIC_B 25

// Analog value store
// NB this is very crude - a global array, and for practical purposes you'll
// probably want to low-pass filter the analog inputs and treat audio, CV and
// pot readings differently
uint16_t analogValues[8] = {0};
// 0 = Audio In 1
// 1 = Audio In 2
// 2 = CV In 1
// 3 = CV In 2
// 4 = Main knob
// 5 = Knob X
// 6 = Knob Y
// 7 = Switch

const char *labels[] = {
    "Audio1",
    "Audio2",
    "CV In 1",
    "CV In 2",
    "Main",
    "Knob X",
    "Knob Y",
    "Switch"};

#define AUDIO_L_IN_1 26
#define AUDIO_R_IN_1 27
#define MUX_IO_1 28
#define MUX_IO_2 29

// DAC parameters
//  A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
//  A-channel, 2x, active -- NB 2x = ~3.27v max, not 4.096
#define DAC_config_chan_A_gain 0b0001000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000
#define DAC_config_chan_B_gain 0b1001000000000000
// SPI configurations
#define PIN_CS 21   // WAS 5
#define PIN_SCK 18  // WAS 6
#define PIN_MOSI 19 // WAS 7
#define SPI_PORT spi0

// Write to the dac - again, super primitive, no error checking
void dacWrite(int channel, int value)
{
    uint16_t DAC_data = (!channel) ? (DAC_config_chan_A_gain | ((value) & 0xffff)) : (DAC_config_chan_B_gain | ((value) & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
}

// Pulse ins and outs
#define PULSE_1_INPUT 2
#define PULSE_2_INPUT 3
#define PULSE_1_RAW_OUT 8
#define PULSE_2_RAW_OUT 9

// PWM CV Output (input via Mux)
#define CV_1_PWM 22
#define CV_2_PWM 23

#define NUM_LEDS 6
#define LED1 10
#define LED2 11
#define LED3 12
#define LED4 13
#define LED5 14
#define LED6 15
uint leds[NUM_LEDS] = {LED1, LED2, LED3, LED4, LED5, LED6};

// Little random number generator from ChatGPT, not sure it works very well
// I just want random() from Arduino
#define LCG_A 1664525
#define LCG_C 1013904223
#define LCG_M (1ULL << 31)  // Using 2^31 as modulus
unsigned long lcg_seed = 1; // Example seed, you might want to initialize this with something variable
// Simple LCG function
unsigned int lcg_rand()
{
    lcg_seed = (LCG_A * lcg_seed + LCG_C) % LCG_M;
    return lcg_seed;
}

// Just blinking the LEDs every 125 milliseconds
bool led_timer_callback(struct repeating_timer *t)
{
    int num = lcg_rand(); // not sure this is right, doesn't seem very random
    for (int x = 0; x < NUM_LEDS; x++)
    {
        bool bit = (num >> x) & 1;
        gpio_put(leds[x], bit);
    }

    // And let's just flash the Pulse outputs while we're here
    gpio_put(PULSE_1_RAW_OUT, (num >> 4) & 1);
    gpio_put(PULSE_2_RAW_OUT, (num >> 5) & 1);

    return true;
}

// set the MUX logic pins - reading channels 0,1,2,3 = 00 01 10 11
void muxUpdate(int num)
{
    gpio_put(MUX_LOGIC_A, ((num >> 0)) & 1);
    gpio_put(MUX_LOGIC_B, ((num >> 1)) & 1);
    // NB this seems to need 1us delay for pins to 'settle' before reading.
}

// Read the Mux IO pins into the value store
void muxRead(int num)
{
    switch (num)
    {

    case 0:

        // read Main Knob
        adc_select_input(2);
        analogValues[4] = adc_read();
        // Read CV 1
        adc_select_input(3);
        analogValues[2] = adc_read();

        break;
    case 1:
        // read Knob X
        adc_select_input(2);
        analogValues[5] = adc_read();
        // Read CV 2
        adc_select_input(3);
        analogValues[3] = adc_read();

        break;
    case 2:
        // read Knob Y
        adc_select_input(2);
        analogValues[6] = adc_read();
        // Read CV 1
        adc_select_input(3);
        analogValues[2] = adc_read();

        break;

    case 3:
        // read Switch
        adc_select_input(2);
        analogValues[7] = adc_read();
        // Read CV 2
        adc_select_input(3);
        analogValues[3] = adc_read();

        break;
    }
}

// Read the audio inputs - presumably if you're really reading audio you'll want DMA
// or something, this is just to ensure the connections are all there
bool analog_timer_callback(struct repeating_timer *t)
{
    // read the Audio Inputs
    adc_select_input(0);
    analogValues[0] = adc_read();
    adc_select_input(1);
    analogValues[1] = adc_read();
}

// The two pulse inputs attached to interrupts here
// Voltage is inverted, so rising = falling
void pulse_callback(uint gpio, uint32_t events)
{
    if ((events & GPIO_IRQ_EDGE_FALL) && (gpio == PULSE_1_INPUT))
    {
        printf("Rising edge detected on Pulse 1\n");
    }
    if ((events & GPIO_IRQ_EDGE_RISE) && (gpio == PULSE_1_INPUT))
    {
        printf("Falling edge detected on Pulse 1\n");
    }

    if ((events & GPIO_IRQ_EDGE_FALL) && (gpio == PULSE_2_INPUT))
    {
        printf("Rising edge detected on Pulse 2\n");
    }
    if ((events & GPIO_IRQ_EDGE_RISE) && (gpio == PULSE_2_INPUT))
    {
        printf("Falling edge detected on Pulse 2\n");
    }
}

// Just print out the analog value store
void printAnalogValues()
{
    printf("Index\tLabel\t\tValue\n");
    printf("-----\t-----\t\t-----\n");

    for (int i = 0; i < 8; i++)
    {
        printf("%d\t%s\t%u\n", i, labels[i], analogValues[i]);
    }
}

// Routine to print out the unique 64-bit identifier on the flash chip on each card
void print_unique_id(uint8_t *id, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        printf("%02X", id[i]);
        if (i < len - 1)
            printf(":");
    }
    printf("\n");
}

int main()
{

    stdio_init_all();
    adc_init(); // Initialize the ADC

    // This random number & unique ID stuff is all just experiments,
    // not sure it's working properly but love the idea

    // sleep_ms(5000); // Wait for the serial port so you can see the printing

    // Create a buffer to hold the unique ID from the flash memory on the card
    uint8_t unique_id[8]; // 64 bits = 8 bytes
    // Retrieve the unique ID
    flash_get_unique_id(unique_id);
    printf("Unique Flash memory ID: ");
    print_unique_id(unique_id, sizeof(unique_id));

    for (int k = 0; k < 4; k++) // use last 4 numbers
    {
        lcg_seed = lcg_seed * unique_id[k];
    }
    printf("Random seed: %d\n", lcg_seed);
    for (int k = 0; k < 10; k++)
    {
        unsigned long bignum = lcg_rand();
        int smallnum = (int)bignum;
        printf("Random number %d = %d or %d\n", k, bignum, smallnum);
    }

    // Set ADC pins
    adc_gpio_init(AUDIO_L_IN_1);
    adc_gpio_init(AUDIO_R_IN_1);
    adc_gpio_init(MUX_IO_1);
    adc_gpio_init(MUX_IO_2);

    // Initialize all LEDs
    for (int i = 0; i < NUM_LEDS; i++)
    {
        gpio_init(leds[i]);
        gpio_set_dir(leds[i], GPIO_OUT);
    }

    // Initialize Mux Control pins
    gpio_init(MUX_LOGIC_A);
    gpio_init(MUX_LOGIC_B);
    gpio_set_dir(MUX_LOGIC_A, GPIO_OUT);
    gpio_set_dir(MUX_LOGIC_B, GPIO_OUT);

    // Initialize Pulse ins and outs
    gpio_init(PULSE_1_INPUT);
    gpio_set_dir(PULSE_1_INPUT, GPIO_IN);
    gpio_pull_up(PULSE_1_INPUT); // NB Needs pullup to activate transistor on inputs

    gpio_init(PULSE_2_INPUT);
    gpio_set_dir(PULSE_2_INPUT, GPIO_IN);
    gpio_pull_up(PULSE_2_INPUT); // NB: Needs pullup to activate transistor on inputs

    // Attach pulse inputs to functions. NB pulse inputs are inverted, so 'EDGE_RISE' triggers 'Fall'
    gpio_set_irq_enabled_with_callback(PULSE_1_INPUT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_callback);
    gpio_set_irq_enabled_with_callback(PULSE_2_INPUT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_callback);

    gpio_init(PULSE_1_RAW_OUT);
    gpio_set_dir(PULSE_1_RAW_OUT, GPIO_OUT);

    gpio_init(PULSE_2_RAW_OUT);
    gpio_set_dir(PULSE_2_RAW_OUT, GPIO_OUT);

    // Initialize PWM CV Outs
    uint slice_CV1 = pwm_gpio_to_slice_num(CV_1_PWM);
    uint channel_CV1 = pwm_gpio_to_channel(CV_1_PWM);
    uint slice_CV2 = pwm_gpio_to_slice_num(CV_2_PWM);
    uint channel_CV2 = pwm_gpio_to_channel(CV_2_PWM);

    gpio_set_function(CV_1_PWM, GPIO_FUNC_PWM);
    gpio_set_function(CV_2_PWM, GPIO_FUNC_PWM);

    pwm_set_wrap(slice_CV1, 2047); // 11 bit 60kHz
    pwm_set_wrap(slice_CV2, 2047);

    pwm_set_chan_level(slice_CV1, channel_CV1, 1023); // 50% of 2047
    pwm_set_chan_level(slice_CV2, channel_CV2, 1023);

    pwm_set_enabled(slice_CV1, true);
    pwm_set_enabled(slice_CV2, true);

    // Initialize hardware timers for LED update and Analog reads
    struct repeating_timer timer;
    struct repeating_timer timer2;
    add_repeating_timer_ms(50, led_timer_callback, NULL, &timer);
    add_repeating_timer_ms(20, analog_timer_callback, NULL, &timer2);

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    while (true)
    {

        // Just some test routines nothing to see here

        // Read input channels 2 & 3 through the 4052 mux
        for (int i = 0; i < 4; i++)
        {
            muxUpdate(i);
            sleep_us(1); // Let the Mux Set itself
            muxRead(i);
        }

        // Knob control over CV outs
        // int a_level = (analogValues[4]) + analogValues[6] / 30;
        // int b_level = (analogValues[5]) + analogValues[6] / 30;

        // // pwm_set_chan_level(slice_CV1, channel_CV1, a_level); // Map to main knob
        // // pwm_set_chan_level(slice_CV2, channel_CV2, b_level); // Map to X knob
        // dacWrite(0, a_level);
        // dacWrite(1, b_level);

        // printf("CV1 1: %d, CV2 2: %d\n", a_level, b_level);
        // sleep_ms(50);

        // Main knob controls 1v steps
        // Some crude calibrations based on one module, haven't yet tested or optimised any of this
        int seq[16] = {0, 50, 215, 381, 546, 711, 877, 1042, 1207, 1373, 1538, 1703, 1869, 2034, 2047, 0};
        int seq2[16] = {0, 49, 215, 381, 546, 712, 877, 1043, 1209, 1374, 1540, 1706, 1871, 2037, 2047, 0};
        int seq3[16] = {4095, 4095, 3032, 2757, 2482, 2207, 1932, 1657, 1382, 1107, 832, 557, 282, 0, 0, 0};

        int s = analogValues[4] >> 8;

        pwm_set_chan_level(slice_CV1, channel_CV1, seq[s]);
        pwm_set_chan_level(slice_CV2, channel_CV2, seq2[s]);
        dacWrite(0, seq3[s]);
        dacWrite(1, seq3[s]);

        printf("Volts = %d\n", s - 7);
        sleep_ms(50);

        // printAnalogValues();
    };

    return 0;
}
