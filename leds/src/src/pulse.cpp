#include <FastLED.h>
#include "constants.h"
#include "pulse.h"

void Pulse::init(CRGB color) {
    pulseColor = rgb2hsv_approximate(color);
}

void Pulse::update() {    
    if (pulseColor.value + step > 255 || pulseColor.value + step < 10)
    {
        step = -step; //reverse direction
    }

    pulseColor.value = pulseColor.value + step; //change the brightness of the LED
    fill_solid(g_leds, NUM_LEDS, pulseColor);
    FastLED.show();
}