#include <Arduino.h>  //include library
#include <FastLED.h> //include library
#include "constants.h"
#include "pulse.h"
#include <WiFi.h>
#include "rainbow.h"

int PIN_ONE = 12;
int PIN_TWO = 14;
int PIN_FOUR = 27;
int PIN_EIGHT = 26;
int PIN_SIXTEEN = 25;
int PIN_THIRTY_TWO = 33;
int PIN_SIXTY_FOUR = 32;
int PIN_ONE_HUNDRED_TWENTY_EIGHT = 35;

int currentCode;

unsigned long startMillis;
unsigned long currentMillis;


CRGB g_leds[NUM_LEDS]; //create our LED array object for all our LEDs
Pulse pulse = Pulse();
Rainbow rb = Rainbow();

void setup()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    FastLED.addLeds<CHIP_SET, DATA_PIN, COLOR_ORDER>(g_leds, NUM_LEDS);
    FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_AMPS);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
}

void amp() //active to signal to human player to active amp
    {
        pulse.init(CRGB(255, 165, 0), 1000, 500);
        pulse.update();
    }

void coop() //activate to signal to human player to active Cooperation bonus
    {
        pulse.init(CRGB(255, 175, 0), 1000, 500);
        pulse.update();
    }
void neutral() //activate to stop signaling 
    {
        pulse.init(CRGB(255, 0, 255))
    }
void red() //turn it all red
    {
        pulse.init(CRGB(255, 0, 0));
        pulse.update();
    }
void blue() //turn it all blue
    {
        pulse.init(CRGB(0,0,255));
        pulse.update();
    }
    void OFF() {
        FastLED.clear();
    }
    void rainbow() 
    {
        rb.init(100);
        rb.update();
    }
    void green()
    {
        pulse.init(CRGB(0,255,0));
        pulse.update();
    }
    void knightkrawler()
    {
        pulse.init(CRGB(128,0,0))
        pulse.update();
    }



     void loop() 
    {
    int code = 0;

    if (digitalRead(PIN_ONE))
    {
        code += 1;
    }

    if (digitalRead(PIN_TWO))
    {
        code += 2;
    }

    if (digitalRead(PIN_FOUR))
    {
        code += 4;
    }

    if (digitalRead(PIN_EIGHT))
    {
        code += 8;
    }

    if (digitalRead(PIN_SIXTEEN))
    {
        code += 16;
    }

    if (digitalRead(PIN_THIRTY_TWO))
    {
        code += 32;
    }

    if (digitalRead(PIN_SIXTY_FOUR))
    {
        code += 64;
    }

    if (digitalRead(PIN_ONE_HUNDRED_TWENTY_EIGHT))
    {
        code += 128;
    }

    if (currentCode != code)
    {
        switch (code)
        {
            case 1:
                amp();
                break;
            case 2:
                coop(); 
                break;
            case 3:
                neutral();
                break;
            case 4:
                red();
                break;
            case 5:
                blue();
                break;
            case 6:
                OFF();
                break;
            case 7:
                rainbow();
                break;
            case 8:
                green();
                break;
            case 9:
                knightkrawler();
                break;
        }
    }
    currentCode = code;

    if (code >=1 || code <=6)
    {
        pulse.update();
    }
    else
    {
        FastLED.show();
    }
    }