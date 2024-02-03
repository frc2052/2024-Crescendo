#ifndef PULSE_H
#define PULSE_H

#include <FastLED.h>

extern CRGB g_leds[];

class Pulse {
    private:
        CRGB pulseColor = CRGB(0, 0, 255); //initialize to white

        unsigned long startPulseMillis;

        int msOn;
        int msOff;

    public:
        void init(CRGB color, int millisOn, int millisOff);
        void update();
};

#endif