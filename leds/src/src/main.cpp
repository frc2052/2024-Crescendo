//This code is just temporary until I we come up with ideas
#include <Arduino.h>  //include library
#include <FastLED.h> //include library
#include "constants.h"
#include "pulse.h"

CRGB g_leds[NUM_LEDS]; //create our LED array object for all our LEDs
Pulse pulseGold= Pulse();

void setup() {
  FastLED.addLeds<CHIP_SET, DATA_PIN, COLOR_ORDER>(g_leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_AMPS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  pulseGold.init(CRGB::Orange);
}

void loop() {
  pulseGold.update();
}