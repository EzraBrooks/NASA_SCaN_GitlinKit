


//#include "Nokia_LCD_functions.h"  //include library to drive NOKIA display
#include <HammingEncDec.h>        // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>      // include the modulator/demodulator functionality
OpticalReceiver phototransistor;  // create an instance of the receiver
byte c; //holds byte returned from receiver

unsigned long  timelastchar;          // variable to hold the time the last valid character was received
bool           blankedvalues;         // flag to determine if values have been blanked for a bad laser link
String         parameterValue;      // holds the measurand being built up character-by-character
String         strTemperature, strPressure, strHumidity; // holds the values of the measurands

void setup() 
{
  Serial.begin(9600);                 // start the serial port on the Arduino
  Serial.println("NASA SCaN Gitlinkit Laser Relay demonstration -- powering on.");
  phototransistor.set_speed(2000);             // laser receive speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  phototransistor.set_rxpin(2);       // pin the phototransistor is connected to
  phototransistor.set_inverted(true);                       // if receive signal is inverted (Laser on = logic 0) set this to true
  phototransistor.begin();                                  // initialize the receiver

}

/* --------- INTERRUPT SERVICE ROUTINE (ISR) ----------
   Set up an interrupt service routine to receive characters
   Arduino Timer2 reads the LIGHT_RECEIVE_PIN at laser
   receive speed to receive each half bit */
ISR(TIMER2_COMPA_vect)
{
  phototransistor.receive();   // check for a received character every timer interrupt
}

void loop() 
{
  bool received = phototransistor.GetByte(c);     // get a character from the laser receiver if one is available
  if (received)
  {
   // if a character is ready, look at it
   Serial.println(c);
    switch (c)
    {       
      // if the character is a terminator, store what was built in a variable and display it
      case 84:         // ASCII T termination character for temperature, use string built to this point for temp
        strTemperature=parameterValue;
        parameterValue="";
        Serial.println(strTemperature);
        break; 
      case 72:        // ASCII H termination character for humidity, use string built to this point for humidity
        strHumidity=parameterValue;
        parameterValue="";
        Serial.println(strHumidity);
        break;
      default :
        parameterValue=parameterValue+=(char)c;  // keep building a string character-by-character until a terminator is found
        break;
    }
  } 
} // end main loop
