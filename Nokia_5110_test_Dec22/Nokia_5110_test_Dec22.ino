/*
 7-17-2011
 Spark Fun Electronics 2011
 Nathan Seidle
 
 This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This code writes a series of images and text to the Nokia 5110 84x48 graphic LCD:
 http://www.sparkfun.com/products/10168
 
 Do not drive the backlight with 5V. It will smoke. However, the backlight on the LCD seems to be 
 happy with direct drive from the 3.3V regulator.
 You will need 5 signal lines to connect to the LCD, 3.3 or 5V for power, 3.3V for LED backlight, and 1 for ground.
 Tweaked by Jimmy Acevedo of GSFC September 2020
 following: https://learn.sparkfun.com/tutorials/graphic-lcd-hookup-guide/discuss
 and https://learn.sparkfun.com/tutorials/graphic-lcd-hookup-guide?_ga=2.208793691.839412505.1600959208-869962234.1599661040
 in "limiting resistors" configuration.
 */

//Pushed all of the lower-level functions back out to the header file for cleanliness. -JA
#include "Nokia_LCD_Functions.h"

void setup(void) 
{
  LCDInit(); //Init the LCD
}


void loop(void) 
{
  LCDClear();
  LCDString("Ad astra per asperam!");
  delay(1500);

  LCDClear();
  LCDString("(Ad astra   GRADATIM.)");
  delay(1500);
}
