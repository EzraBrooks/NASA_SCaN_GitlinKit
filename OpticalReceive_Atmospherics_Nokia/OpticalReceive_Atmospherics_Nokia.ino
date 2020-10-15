/***************************************************************************
  Laser Receiver Code

  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin 
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication

  Arduino sketch by Tom Gitlin; modified slightly by J. Acevedo
  version 1.1 07 Aug 2018 -- Just tweaking style conventions. Preparing to build in true relaying. -JA

***************************************************************************/
#include "Nokia_LCD_functions.h" //include library to drive NOKIA display
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
bool           displayChanged;         //checks to clear screen if display has changed 
                                      
/* IMPORTANT: The Delay function is used sparingly because it stalls program execution
(except for interrupts) and can affects the ability to check if characters are ready. */
                                         
String         parameterValue;      // holds the measurand being built up character-by-character
String         strTemperature, strPressure, strHumidity, displayTemp, displayHumid; // holds the values of the measurands
char           tempChar;
char           charsTemp[10], charsHumid[10];
const int      LED_Temp = 6, LED_Press = 5, LED_Humid = 4;  // Pins for the Temp, Press, and Humidity LEDs
const int      BUTTON_INPUT = 3; //Pin for button press input.




// Pin value for the phototransistor input
const int      PHOTOTRANSISTOR_RECEIVE = 2;


const int      DISPLAY_NUMBER_OF_DISPLAYS = 1;

OpticalReceiver phototransistor;  // create an instance of the receiver


void setup()
{
  Serial.begin(9600);                 // start the serial port on the Arduino
  
  phototransistor.set_speed(laserreceivespeed);             // laser receive speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  phototransistor.set_rxpin(PHOTOTRANSISTOR_RECEIVE);       // pin the phototransistor is connected to
  phototransistor.set_inverted(true);                       // if receive signal is inverted (Laser on = logic 0) set this to true
  phototransistor.begin();                                  // initialize the receiver

  LCDInit(); //Init the LCD
  LCDString("INITIALIZING DISPLAY");
  delay(3000);
  LCDClear();
  
  
}

// Set up an interrupt service routine to receive characters
// Arduino Timer2 reads the LIGHT_RECEIVE_PIN at laser receive speed to receive each half bit
ISR(TIMER2_COMPA_vect)
{
  phototransistor.receive();   // check for a received character every timer interrupt
}

void loop()
{
  displayChanged = false;
  LCDString("WAITING FOR LASER");
  delay(3000);
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
   Serial.println("LINK IS GOOD");
   delay(3000);
   
   //goToXY(10, 30);
   if (displayChanged){
      LCDClear() ;
   }
   LCDString("LINK IS GOOD");

   // cycle through displaying the values if the link is good
   if (timetoswitch)
   {
    timenow=millis();
     switch (measurand)
     {
       case 1:
         displayTemp = strTemperature + "F";
         int str_len = displayTemp.length() + 1; 
 
         // Prepare the character array (the buffer) 
         char char_array[str_len];
           
         // Copy it over 
         displayTemp.toCharArray(char_array, str_len);
         LCDString(char_array);
         //LCDString(strTemperature.toCharArray());
         digitalWrite(LED_Temp, HIGH);
         break;
       case 2:
         //LCDString(strHumidity);
         displayHumid = strHumidity + " %h";
         digitalWrite(LED_Humid, HIGH);
         break;
      }
    }

    // determine if it is time to switch the display to the next measurand
    if(millis() >= (timenow+displayperiod))
    {
      LCDClear();
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
  } //end IFCHECK (link good?)
  else
  {
  // link is bad, so display a distinctive pattern,
  // turn off the individual LEDs, and blank out the measurands
    Serial.println("LINK IS BAD");
    if (displayChanged) {
      LCDClear();
    }
    LCDString("LASER NOT CONNECTED");
    delay(3000);
    if (!blankedvalues)
    {
      blankedvalues=true;
      digitalWrite(LED_Temp, LOW);
      digitalWrite(LED_Humid, LOW); 
      strTemperature = " ";
      strPressure    = " ";
      strHumidity    = " ";
      LCDClear();
      LCDString("LASER");
    }
  } //end IFCHECK-ELSE (link is bad)
  LCDClear();
} // end main loop

// This function is written to drive a NOKIA display
void displayValue(String msgtodisplay)
{     
  int i, strlength, decimalatposition;
  char t;
  bool printeddecimal;

  strlength=msgtodisplay.length();

  if ((strlength>0))   // only display a message if the message length is greater than 0 and less than 9
  {
    //Serial.print(msgtodisplay); Serial.print(" Length: "); Serial.print(strlength);
    //Serial.print(" Decimal: "); Serial.println(decimalatposition);

      for (i=0;i<strlength;i++)
      {
         //lc.setChar(0,strlength-2-i,msgtodisplay.charAt(i),false);
         LCDCharacter(msgtodisplay.charAt(i));
      }
//    LCDString(msgtodisplay);
  }
  else 
  {
    // display an error message on the display
    // if the message is invalid
    LCDString("-ERROR-");
  }
}
