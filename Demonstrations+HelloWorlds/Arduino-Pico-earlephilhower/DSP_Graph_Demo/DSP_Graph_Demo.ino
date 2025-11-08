#include <SPI.h>


#include "dspatch/include/DSPatch_Embedded.h"

bool core1_separate_stack = true;

int ledon=1;
struct repeating_timer audioTimer;
double sampleIndex=0;

#define DAC_config_chan_A_gain 0b0001000000000000
#define DAC_config_chan_B_gain 0b1001000000000000
#define PIN_CS 21


class ADC0 final : public DSPatch::Component
{
public:
    ADC0()
        // the order in which buffers are Process_()'ed is not important
        : Component( ProcessOrder::OutOfOrder )
    {
        SetInputCount_(0);
        SetOutputCount_( 1 );
    }


protected:
    void __not_in_flash_func(Process_)( DSPatch::SignalBus&, DSPatch::SignalBus& outputs ) override
    {
      const float signal = (analogRead(A0) - 2048) / 2048.f;
      outputs.SetValue( 0, signal);
    }
private:
};

class RingMod final : public DSPatch::Component
{
public:
    RingMod()
        // the order in which buffers are Process_()'ed is not important
        : Component( ProcessOrder::OutOfOrder )
    {
        SetInputCount_(2);
        SetOutputCount_( 1 );
    }

    void SetGains(float gainCh0, float gainCh1) {
      gain0 = gainCh0;
      gain1 = gainCh1;
    }

protected:
    void __not_in_flash_func(Process_)( DSPatch::SignalBus& inputs, DSPatch::SignalBus& outputs ) override
    {
      const float sig0 = *inputs.GetValue<float>(0) * gain0;
      const float sig1 = *inputs.GetValue<float>(1) * gain1;

      outputs.SetValue( 0, sig0*sig1);
    }
private:
  float gain0=1, gain1=1;
};

class SawOsc final : public DSPatch::Component
{
public:
    SawOsc()
        // the order in which buffers are Process_()'ed is not important
        : Component( ProcessOrder::OutOfOrder )
    {
        // add 1 output
        SetOutputCount_( 1 );
    }

    void setFrequency(float inc) {
      increment = inc;
    }

protected:
    void __not_in_flash_func(Process_)( DSPatch::SignalBus&, DSPatch::SignalBus& outputs ) override
    {
      phase+=increment;
      if (phase >= 1.0f) {
        phase -= 2.0f;
      }
      outputs.SetValue( 0, phase);
    }
private:
  float phase=0.0;
  float increment = 0.01f;
};

class DAC final : public DSPatch::Component
{
public:
    DAC()
        // the order in which buffers are Process_()'ed is not important
        : Component( ProcessOrder::OutOfOrder )
    {
        SetOutputCount_( 0 );
        SetInputCount_(2);
    }

protected:
    void __not_in_flash_func(Process_)( DSPatch::SignalBus& inputs, DSPatch::SignalBus&) override
    {
      //overhead is approx 4us at 225 MHz, -O3
      int sigL, sigR;
      sigL = static_cast<int>(*inputs.GetValue<float>(0) * 2048.f) + 2047;
      sigR = static_cast<int>(*inputs.GetValue<float>(1) * 2048.f) + 2047;

      // dacWrite(sigL, sigR);
      //assuming SPI already set up, so no need for begin/end transaction
      //CS is hardware controlled
      const uint16_t DAC_dataL = DAC_config_chan_A_gain | (sigL & 0xFFF);
      const uint16_t DAC_dataR = DAC_config_chan_B_gain | (sigR & 0xFFF);
      uint16_t ret;
      hw_write_masked(&spi_get_hw(spi0)->cr0, (16 - 1) << SPI_SSPCR0_DSS_LSB, SPI_SSPCR0_DSS_BITS); // Fast set to 16-bits
      spi_write16_read16_blocking(spi0, &DAC_dataL, &ret, 1);
      hw_write_masked(&spi_get_hw(spi0)->cr0, (16 - 1) << SPI_SSPCR0_DSS_LSB, SPI_SSPCR0_DSS_BITS); // Fast set to 16-bits
      spi_write16_read16_blocking(spi0, &DAC_dataR, &ret, 1);
    }
private:
};

auto circuit = std::make_shared<DSPatch::Circuit>();

auto sawOsc1 = std::make_shared<SawOsc>();
auto DAC1 = std::make_shared<DAC>();
auto ADC0_1 = std::make_shared<ADC0>();
auto RM1 = std::make_shared<RingMod>();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  __wfi();
}



long ts;
long dsptime;
bool __isr audio_timer_callback(__unused struct repeating_timer *t) {
  sampleIndex += 1;
  if (sampleIndex >= 1000) {
    digitalWrite(10, ledon);
    ledon = 1 - ledon;
    sampleIndex=0;
  }

  ts = micros();

  circuit->Tick();

  dsptime = micros() - ts;

  return true;
}

void setup1() {

  // Setup SPI
  SPI.setCS(PIN_CS);
  SPI.begin(1); // Initialize the SPI bus

  //pass settings to SPI class
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0)); // Start SPI transaction
  SPI.endTransaction();

  pinMode(10, OUTPUT);
  digitalWrite(10,1);

  //set up a test circuit that ring mods ADC0 with a saw wave oscillator
  circuit->AddComponent( sawOsc1 );
  circuit->AddComponent(ADC0_1);
  circuit->AddComponent( DAC1 );
  circuit->AddComponent( RM1 );

  circuit->ConnectOutToIn( ADC0_1, 0, RM1, 0 );
  circuit->ConnectOutToIn( sawOsc1, 0, RM1, 1 );

  RM1->SetGains(3,1);


  circuit->ConnectOutToIn( RM1, 0, DAC1, 0 );
  circuit->ConnectOutToIn( RM1, 0, DAC1, 1 );

  //start audio
  int samplePeriod = 45; //in uS
  add_repeating_timer_us(-samplePeriod, audio_timer_callback, NULL, &audioTimer);

}

void loop1() {
  // __wfi();
  Serial.println(dsptime);
  delay(500);

}
