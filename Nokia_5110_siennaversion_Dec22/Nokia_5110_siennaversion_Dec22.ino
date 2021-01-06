/***************************************************************************
  Laser Receiver Code
  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin 
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication
  Arduino sketch by Tom Gitlin; modified slightly by J. Acevedo
  version 1.3 22 Dec 2020 -- Modifying to accommodate Nokia 5110 display.

  This sketch is expecting:
  Nokia 5110 pins:      Arduino pins:
  1                     3.3V
  2                     GND
  3                     7, via 1k R
  4                     6, via 10k R
  5                     5, via 10k R
  6                     11, via 10k R
  7                     13, via 10k R
  8                     (optional) 2.4-5V 
  
***************************************************************************/
#include "Nokia_LCD_functions.h" //include library to drive NOKIA display
#include <HammingEncDec.h>        // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>      // include the modulator/demodulator functionality

// new packet structure by Dan "Memes" Koris
typedef union packet
{
    char bytes[10];
    struct 
    {
        byte  starter      = '\x01';
        float temperature  = 0.0f;
        float humidity     = 0.0f;
        byte  terminator   = '\x04';
    }; 
};


int            linktimeout = 1000;    // if no valid characters received in this time period, assume the link is bad
int            displayperiod = 2500;  // how long to keep a number displayed (in ms, 1000 ms = 1 second)
int            laserreceivespeed = 2000; //MUST MATCH THE SPEED AS DEFINED IN THE OPTICALTRANSMIT CODE

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
int            receive_count = 0; // global
union          packet pkt; // global receive buffer
bool           valid_read = false; // global to indicate the quality of our data.

                                      
/* IMPORTANT: The Delay function is used sparingly because it stalls program execution
(except for interrupts) and can affects the ability to check if characters are ready. */
                                         
String         parameterValue;      // holds the measurand being built up character-by-character
String         strTemperature, strPressure, strHumidity, displayTemp, displayHumid; // holds the values of the measurands
char           tempChar;
char           charsTemp[10], charsHumid[10];
// DECONFLICT THESE PINS LATER -- they walk over pins reserved in the header file.
//const int      LED_Temp = 6, LED_Press = 5, LED_Humid = 4;  // Pins for the Temp, Press, and Humidity LEDs
const int      BUTTON_INPUT = 3; //Pin for button press input.

// Pin value for the phototransistor input
const int      PHOTOTRANSISTOR_RECEIVE = 2;
const int      DISPLAY_NUMBER_OF_DISPLAYS = 1;

OpticalReceiver phototransistor;  // create an instance of the receiver


void setup()
{
  Serial.begin(9600);                 // start the serial port on the Arduino
  Serial.print("Size of union-packet: ");
  Serial.println(sizeof(union packet));  
  phototransistor.set_speed(laserreceivespeed);             // laser receive speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  phototransistor.set_rxpin(PHOTOTRANSISTOR_RECEIVE);       // pin the phototransistor is connected to
  phototransistor.set_inverted(true);                       // if receive signal is inverted (Laser on = logic 0) set this to true
  phototransistor.begin();                                  // initialize the receiver

  LCDInit(); //Init the LCD
  LCDString("INITIALIZING DISPLAY");
  delay(1700);
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
  //Serial.print("T: \t");  Serial.print(pkt.temperature);
  //Serial.print("\t H: \t"); Serial.println(pkt.humidity);
  //LCDString("WAITING FOR LASER");
  // block here waiting for a byte to be received
    byte received_byte;
    bool good_data_read = phototransistor.GetByte(received_byte);
 
    if (good_data_read)
    {
      if (valid_read)
      {
          pkt.bytes[receive_count++] = received_byte;
          Serial.print("Received: \t"); Serial.println(received_byte);
  
          if (pkt.starter    == '\x01' &&
              pkt.terminator == '\x04')
          {
              // COMPLETE PACKET HAS ARRIVED
              Serial.print("Temp: \t");
              Serial.println(pkt.temperature);
              Serial.print("Humi: \t");
              Serial.println(pkt.humidity);
              LCDClear();
              String foobar = "Temp: " + String(pkt.temperature) + '\n';
              String foobar2 = "Humi: " + String(pkt.humidity);
              LCDString(foobar.c_str());
              LCDString(foobar2.c_str());
              reset_packet();
          }
          else if (receive_count > sizeof(union packet) + 1)
          {
              reset_packet();
              Serial.println("Packet's full; reset packet.");
          }
      }
      else
      {
          if (received_byte == '\x01')
          {
              Serial.println("Starting new packet. I'm excited!");
              valid_read = true;
              pkt.bytes[receive_count++] = received_byte;
          }
          //Serial.println("INVALID READ");
      }
    }
} // end main loop

void reset_packet()
{
    valid_read    = false;
    receive_count = 0;
    memset(&pkt, 0, sizeof(union packet));
}
