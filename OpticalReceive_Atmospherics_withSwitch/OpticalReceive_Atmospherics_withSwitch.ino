/***************************************************************************
  Laser Receiver Code

  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin 
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication

  Arduino sketch by Tom Gitlin; modified slightly by J. Acevedo
  version 1.1 07 Aug 2018 -- Just tweaking style conventions. Preparing to build in true relaying. -JA

***************************************************************************/

#include "LedControl.h"           // include the library to drive the MAX72XX LED Control
#include <HammingEncDec.h>        // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>      // include the modulator/demodulator functionality

int            linktimeout = 1000;    // if no valid characters received in this time period, assume the link is bad
int            displayperiod = 2500;  // how long to keep a number displayed (in ms, 1000 ms = 1 second)
int            laserreceivespeed = 2000; //MUST MATCH THE SPEED AS DEFINED IN THE OPTICALTRANSMIT CODE

/***************************************************************************
  You can display some 'characters' on the 7 segment display by controlling
  individual segments.  
  
  DP - A - B - C - D - E - F - G    <- LED Segment designations
  128  64  32  16  8   4   2   1    <- Value
  
  Segments are labled  A through G starting at the top and going clockwise: 
     
      A
      _
   F | | B
      -        <-G (center LED)
   E |_| C  .  <- Decimal point
      D
      
   For example, to display an "E", light segments A, D, E, F and G.
   To do this, the .setrow function of the LedControl library would
   be used to send a value of 79 Decimal, 4F Hex (or 01001111 binary) 
***************************************************************************/

byte           c;                     // variable to hold the byte returned from the receiver
int            i;                     // loop counter
int            j;                     // alternate loop counter for button-press (going to pull it off the cyclic timer and replace with button)
int            measurand;             // tracks the number of measurands (3 for the demo, Temp, Press and Humidity)
unsigned long  timelastchar;          // variable to hold the time the last valid character was received
bool           linkgood;              // flag to store optical link status
bool           timetoswitch;          // flag to cycle through the measurands
bool           blankedvalues;         // flag to determine if values have been blanked for a bad laser link
unsigned long  timenow;               // holds the Arduino running time in milliseconds for display times
                                      
/* IMPORTANT: The Delay function is used sparingly because it stalls program execution
(except for interrupts) and can affects the ability to check if characters are ready. */
                                         
String         parameterValue;      // holds the measurand being built up character-by-character
String         strTemperature, strPressure, strHumidity; // holds the values of the measurands
char           tempChar;
const int      LED_Temp = 6, LED_Press = 5, LED_Humid = 4;  // Pins for the Temp, Press, and Humidity LEDs
const int      PIN_SWITCH_a = 3; 
const int      PIN_SWITCH_b = 2; 
bool           switch_a = 0;
bool           switch_b = 0;

// Pin value for the phototransistor input
const int      PHOTOTRANSISTOR_RECEIVE = 7;

// Pin values for MAX72XX 8-digit LED display:
const int      DISPLAY_DATAIN = 12;    
const int      DISPLAY_CLOCK  = 11;
const int      DISPLAY_LOAD   = 10;
const int      DISPLAY_NUMBER_OF_DISPLAYS = 1;
LedControl     lc = LedControl(DISPLAY_DATAIN, DISPLAY_CLOCK, DISPLAY_LOAD, DISPLAY_NUMBER_OF_DISPLAYS);
OpticalReceiver phototransistor;  // create an instance of the receiver


void setup()
{
  Serial.begin(9600);                 // start the serial port on the Arduino
  
  phototransistor.set_speed(laserreceivespeed);             // laser receive speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  phototransistor.set_rxpin(PHOTOTRANSISTOR_RECEIVE);       // pin the phototransistor is connected to
  phototransistor.set_inverted(true);                       // if receive signal is inverted (Laser on = logic 0) set this to true
  phototransistor.begin();                                  // initialize the receiver

  // The MAX72XX is in power-saving mode on startup, so wake it up
  lc.shutdown(0,false);
  // Set the brightness to a medium value
  lc.setIntensity(0,4);
  // clear the display
  lc.clearDisplay(0);

  // define the pins to be used for the LEDs that indicate which measurand
  // is being displayed
  pinMode(LED_Temp, OUTPUT);
  pinMode(LED_Humid, OUTPUT);
  pinMode(PIN_SWITCH_a, INPUT);
  pinMode(PIN_SWITCH_b, INPUT);
}

// Set up an interrupt service routine to receive characters
// Arduino Timer2 reads the LIGHT_RECEIVE_PIN at laser receive speed to receive each half bit
ISR(TIMER2_COMPA_vect)
{
  phototransistor.receive();   // check for a received character every timer interrupt
}

void loop()
{
  c = phototransistor.GetByte();     // get a character from the laser receiver if one is available
  
  if (c>0)
  {
   // if a character is ready, look at it
   Serial.println(c);
   //tempChar=(char)c;
   //Serial.println(tempChar);
    blankedvalues=false;
    timelastchar=millis();
    switch (c)
	  {       
	    // if the character is a terminator, store what was built in a variable and display it
      case 84:         // ASCII T termination character for temperature, use string built to this point for temp
        strTemperature=parameterValue;
        parameterValue="";
        Serial.println(strTemperature);
        //displayValue(strTemperature);
        break;
      case 80:        // ASCII P termination character for pressure, use string built to this point for pressure
        strPressure=parameterValue;
        parameterValue="";
        Serial.println(strPressure);
        //displayValue(strPressure);
        break;    
      case 72:        // ASCII H termination character for humidity, use string built to this point for humidity
        strHumidity=parameterValue;
        parameterValue="";
        Serial.println(strHumidity);
        //displayValue(strHumidity);
        break;
      default :
        //Serial.println(parameterValue);
        parameterValue=parameterValue+=(char)c;  // keep building a string character-by-character until a terminator is found
    }
  }

 linkgood = !(millis() > (timelastchar + linktimeout));  // update the link status based on the timeout value

 if (linkgood)
 {
   switch_a = digitalRead(PIN_SWITCH_a);
   switch_b = digitalRead(PIN_SWITCH_b);
   Serial.print("A: " + (String)switch_a + "\t");
   Serial.println("B: " + (String)switch_b);
   if(!(switch_a) && (switch_b))
   {
     displayValue(strTemperature);
     digitalWrite(LED_Temp, HIGH);
   }
   if(!(switch_b) && (switch_a))
   {
     displayValue(strHumidity);
     digitalWrite(LED_Humid, HIGH);    
   }
   if(switch_a && switch_b)
   {
     // cycle through displaying the values if the link is good
     if (timetoswitch)
     {
      timenow=millis();
       switch (measurand)
       {
         case 1:
           displayValue(strTemperature);
           digitalWrite(LED_Temp, HIGH);
           break;
         case 2:
           displayValue(strHumidity);
           digitalWrite(LED_Humid, HIGH);
           break;
        }
      }
      // determine if it is time to switch the display to the next measurand
      if(millis() >= (timenow+displayperiod))
      {
        lc.clearDisplay(0);
        digitalWrite(LED_Temp, LOW);
        digitalWrite(LED_Humid, LOW);
        timetoswitch = true;
        measurand++;
        if (measurand > 2)
        {
          measurand=1;
        }
      } 
      else 
      {
        timetoswitch = false;
      }
   }
 } //end IFCHECK (link good?)
 else
 {
 // link is bad, so display a distinctive pattern,
 // turn off the individual LEDs, and blank out the measurands
   if (!blankedvalues)
   {
      blankedvalues=true;
      digitalWrite(LED_Temp, LOW);
      digitalWrite(LED_Humid, LOW); 
      strTemperature = " ";
      strPressure    = " ";
      strHumidity    = " ";
      lc.clearDisplay(0);
      lc.setRow(0,5,0x0E); // diplay a capital L
      lc.setRow(0,4,0x77); // diplay a capital A
      lc.setRow(0,3,0x5B); // diplay a lowercase S
      lc.setRow(0,2,0x4F); // diplay a capital E
      lc.setRow(0,1,0x05); // diplay a lowercase r
    }
  } //end IFCHECK-ELSE (link is bad)
} // end main loop

// This function is written to drive an 8 digit LED display, MAX72XX series
void displayValue(String msgtodisplay)
{     
  int i, strlength, decimalatposition;
  char t;
  bool printeddecimal;

  strlength=msgtodisplay.length();

  if ((strlength>0) && (strlength<9))   // only display a message if the message length is greater than 0 and less than 9
  {
    // see if a decimal point is in the message to display
    decimalatposition = msgtodisplay.indexOf('.');
    //Serial.print(msgtodisplay); Serial.print(" Length: "); Serial.print(strlength);
    //Serial.print(" Decimal: "); Serial.println(decimalatposition);
    
    // if there is a decimal, it needs to be treated specially
    if (decimalatposition>0) 
    {
      for (i=0;i<(strlength-1);i++)
      {
        if (i<=(decimalatposition-2))
        {
          lc.setChar(0,strlength-2-i,msgtodisplay.charAt(i),false);
        } 
        else if (i==(decimalatposition-1))
        {
          //Serial.print("Turning on decimal ... ");Serial.print(i);
          lc.setChar(0,strlength-2-i,msgtodisplay.charAt(i),true);
        } 
        else 
        {
          //Serial.print("Display beyond decimal ...");
          lc.setChar(0,strlength-2-i,msgtodisplay.charAt(i+1),false);
        }           
      }//end of FOR loop (i iterating through string length)
    }//end of IFCHECK decimal at position 
    else
    {
      for (i=0;i<strlength;i++)
      {
         //lc.setChar(0,strlength-2-i,msgtodisplay.charAt(i),false);
         lc.setChar(0,strlength-i-1,msgtodisplay.charAt(i),false);
      }
    }
  } 
  else 
  {
    // display an error message on the display
    // if the message is invalid
    lc.setRow(0,6,0x01); // diplay a dash
    lc.setRow(0,5,0x4F); // diplay a capital   E
    lc.setRow(0,4,0x05); // diplay a lowercase r
    lc.setRow(0,3,0x05); // diplay a lowercase r
    lc.setRow(0,2,0x1D); // diplay a lowercase o
    lc.setRow(0,1,0x05); // diplay a lowercase r
    lc.setRow(0,0,0x01); // diplay a dash     
  }
}


