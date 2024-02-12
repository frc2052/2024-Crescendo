#ifndef PULSE_H
#define PULSE_H

#include <FastLED.h>

extern CRGB g_leds[];

class Pulse {
    private:
        int step = -1;
        CHSV pulseColor = CHSV(0, 0, 255); //initialize to white

    public:
        void init(CRGB color);
        void update();
};

#endif