#include "hardware/flash.h"
#define NUM_LEDS 6
#define LED1 10
#define LED2 11
#define LED3 12
#define LED4 13
#define LED5 14
#define LED6 15
uint8_t leds[NUM_LEDS] = {LED1, LED2, LED3, LED4, LED5, LED6};
uint8_t randoms[NUM_LEDS] = {0, 0, 0, 0, 0, 0};
uint8_t outs[NUM_LEDS] = {8, 9, 22, 23, 9, 23};


// Delay between steps
#define STEP_DELAY 20

uint32_t process_unique_id() {
  uint8_t unique_id[8]; // Array to store the 64-bit ID
  flash_get_unique_id(unique_id); // Retrieve the unique ID

  // Cast bytes to uint32_t and combine
  uint32_t high = (unique_id[0] << 24) | (unique_id[1] << 16) | (unique_id[2] << 8) | unique_id[3];
  uint32_t low = (unique_id[4] << 24) | (unique_id[5] << 16) | (unique_id[6] << 8) | unique_id[7];

  // XOR high and low parts to mix the bits
  return high ^ low;
}


uint32_t num = 0;
void setup() {
  num = process_unique_id();
  randomSeed(num);

  // Initialize all LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(leds[i], OUTPUT);
    randoms[i] = random(65535);
  }
  analogWriteRange(256);
}



void loop() {

  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t t = num + (i * randoms[i]) ;
    digitalWrite(leds[i], (t % 24 < 11));
  }
  delay(10);
  num++;
}
