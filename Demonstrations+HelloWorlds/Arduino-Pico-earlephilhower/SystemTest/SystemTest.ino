/*
  Music Thing Modular
  Computer
  System Check
*/

#include "ResponsiveAnalogRead.h"
const int adcResolutionBits = 12;
int maxRange = 1 << adcResolutionBits;

int leds[6] = {10, 11, 12, 13, 14, 15};

const int channelCount = 8; // three knobs, 1 switch, 2 CV
ResponsiveAnalogRead *analog[channelCount];

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
  "Switch"
};

// ADC input pins
#define AUDIO_L_IN_1 26
#define AUDIO_R_IN_1 27
#define MUX_IO_1 28
#define MUX_IO_2 29

// Mux pins
#define MUX_LOGIC_A 24
#define MUX_LOGIC_B 25

#define PULSE_1_INPUT 2
#define PULSE_2_INPUT 3

boolean pulse1 = 0;
boolean pulse2 = 0;



void setup() {

  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  analogReadResolution(adcResolutionBits);

  for (int i = 0; i < 6; i++) {
    pinMode(leds[i], OUTPUT);
  }

  pinMode(MUX_LOGIC_A, OUTPUT);
  pinMode(MUX_LOGIC_B, OUTPUT);

  pinMode(PULSE_1_INPUT, INPUT_PULLUP);
  pinMode(PULSE_2_INPUT, INPUT_PULLUP);


  // Set up the 8 analog channels in Responsive Analog Read.
  // NB this is very crude - values more suited for knobs than CV and certainly not for audio

  for (int i = 0; i < channelCount; i++) {
    analog[i] = new ResponsiveAnalogRead(0, true, .1); //  lower is smoother
    analog[i]->setActivityThreshold(16); // sleep threshold
    analog[i]->setAnalogResolution(1 << adcResolutionBits);
  }

}

int muxCount = 0;

void loop() {

  // Blink LEDS
  //  for (int i = 0; i < 6; i++) {
  //
  //    Serial.println("hello world");
  //    digitalWrite(leds[i], HIGH);   // turn the LED on (HIGH is the voltage level)
  //    delay(100);                       // wait for a second
  //    digitalWrite(leds[i], LOW);    // turn the LED off by making the voltage LOW
  //    delay(100);  // wait for a second
  //  }

  // Read analog 'audio' inputs
  analog[0]->update(maxRange - analogRead(A0)); // inverted
  analog[1]->update(maxRange - analogRead(A1)); // inverted

  pulse1 = !digitalRead(PULSE_1_INPUT);
  pulse2 = !digitalRead(PULSE_2_INPUT);

  // Blink LEDS in line with  inputs
  analogWrite(leds[0], analog[4]->getValue() >> 4); // Main Knob 
  analogWrite(leds[1], analog[5]->getValue() >> 4); // Knob X 
  analogWrite(leds[2], analog[6]->getValue() >> 4); // Knob Y
  analogWrite(leds[3], analog[7]->getValue() >> 4); // Switch Z 
  digitalWrite(leds[4], pulse1); // Pulse 1
  digitalWrite(leds[5], pulse2); // Pulse 2


  muxRead(muxCount);
  muxCount++;
  if (muxCount >= 4) muxCount = 0;
  muxUpdate(muxCount); // If results are odd, put a delay after mux Update

  for (int i = 0; i < channelCount; i++) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(labels[i]);
    Serial.print(": ");
    Serial.println(analog[i]->getValue());
  }
  Serial.println("----");
  delay(5);

}
