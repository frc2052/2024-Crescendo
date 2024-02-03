#ifndef CONST_H
#define CONST_H

#define NUM_LEDS 512 //how many LEDs are in your strip/panel
#define DATA_PIN 2 //signal wire is connected to this pin on arduino
#define COLOR_ORDER GRB //surprisingly, many LED strips don't use the color order Red,Green,Blue
#define CHIP_SET WS2812B //KnightKrawler uses WS2812B Leds, but there are many other options
#define BRIGHTNESS 255 //255 is maximum brightness, lower this value to limit brightness
#define VOLTS 5 //All WS2812B LED strips are 5v
#define MAX_AMPS 2500 //this value is in milliamps, our power converter allows for 3 amp at 5v. Choose a value at or bellow 3000 milliamps

#endif