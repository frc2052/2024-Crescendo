#ifndef RAINBOW_H
#define RAINBOW_H

#include <FastLED.h>
extern CRGB g_leds[];

class Rainbow
{
private:
    int waitMS = 0;
    int step = 1;
    CHSV rainbowColor = CHSV(0, 255, 255); //initialize to red

public:
    void init(int delay){
        waitMS = delay;
    }

    void update(){    
        if (rainbowColor.hue + step > 255)
        {
            rainbowColor.hue = 0;
        }
        else
        {
            rainbowColor.hue = rainbowColor.hue + step;
        }

        fill_solid(g_leds, NUM_LEDS, rainbowColor);
        FastLED.show();
        delay(waitMS);
    }
};

#endif