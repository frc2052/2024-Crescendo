#include <FastLED.h>
#include "constants.h"
#include "pulse.h"

void Pulse::init(CRGB color, int millisOn, int millisOff)
{
    pulseColor = color;
    startPulseMillis = millis();
    msOn = millisOn;
    msOff = millisOff;
}

void Pulse::update()
{    
    if (millis() - startPulseMillis < msOn)
    {
        fill_solid(g_leds, NUM_LEDS, pulseColor);
    }
    else if (millis() - startPulseMillis < msOn + msOff)
    {
        FastLED.clear();
    }
    else
    {
        startPulseMillis = millis();
    }
    FastLED.show();
}