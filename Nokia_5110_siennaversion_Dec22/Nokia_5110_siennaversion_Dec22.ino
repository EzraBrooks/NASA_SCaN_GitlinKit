/***************************************************************************
  Laser Receiver Code
  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication
  Arduino sketch by Tom Gitlin; modified by Jimmy Acevedo.
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
  8                     (optional) 2.4-5V via 330 R

  *** IMPORTANT: The Delay function is used sparingly because it stalls program execution
  (except for interrupts) and can affect the ability to check if characters are ready. ***

*************************************************************************** */
#include "Nokia_LCD_functions.h"  //include library to drive NOKIA display
#include <HammingEncDec.h>        // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>      // include the modulator/demodulator functionality

/* --------- PRE-SETUP ---------- */

/* new packet structure by Dan "Memes" Koris - m241dan */
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

int            linktimeout      = 1000; // if no valid characters received in this time period, assume the link is bad
static int     MODULATE_SPEED   = 2000; // laser modulation speed - should be 500+ bits/second, nominal 2000 (=2KHz)
                                        // MUST MATCH THE SPEED AS DEFINED IN THE OPTICALTRANSMIT CODE
const int      PHOTOTRANSISTOR_RECEIVE = 2; // Pin value for the phototransistor input
byte           c;                       // variable to hold the byte returned from the receiver
unsigned long  timelastchar;            // variable to hold the time the last valid character was received
bool           linkgood;                // flag to store optical link status
bool           displayChanged;          // checks to clear screen if display has changed
unsigned long  timenow;                 // holds the Arduino running time in milliseconds for display times
int            receive_count    = 0;    // global to count received packets
union          packet pkt;              // global receive buffer
bool           valid_read = false;      // global to indicate the quality of our data.
String         strTemper, strHumidi;    // holds the values of the measurands
float          temperature_F = 0.0f;    // variable for a Fahrenheit conversion
String         strTemperFahren;         // string for a Fahrenheit conversion
unsigned int short ellipsiscounter = 0; // iterator for an ellipsis text animation
const int      ELLIPSIS_LOOP_DURATION = 72; // semi-arbitrary cycle size for the ellipsis text animation. Tune as desired.

const unsigned char nasa_worm_BMP [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x70, 0x20, 0x20, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF8, 0xFC, 0xFE, 0xFE, 0xFE, 0x7E, 0x3C,
0x08, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x14, 0x08,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60,
0x80, 0x00, 0x00, 0x00, 0x00, 0x82, 0xC5, 0xE2, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x01, 0x03, 0x05,
0x05, 0x84, 0xC4, 0xE2, 0xE1, 0xE0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,
0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0,
0x81, 0x01, 0x02, 0x0C, 0x02, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x01, 0x03, 0x07,
0x7F, 0xFF, 0xFF, 0xF8, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
0x00, 0x00, 0xC0, 0xF8, 0xFF, 0xFF, 0xFF, 0x1F, 0x03, 0x03, 0x0F, 0x7F, 0xFF, 0xFF, 0xFC, 0xE0,
0x80, 0x00, 0x00, 0x7E, 0xFF, 0xFF, 0xFF, 0xC3, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
0x81, 0x81, 0x81, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xFE, 0xFF, 0xFF, 0x7F, 0x0F,
0x03, 0x07, 0x3F, 0xFF, 0xFF, 0xFE, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3F, 0xFF, 0xFF, 0xFE, 0xE0, 0x80, 0x80, 0xC0, 0xFF, 0xFF,
0xFF, 0xFF, 0x00, 0x00, 0xF0, 0xFE, 0xFF, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x03, 0x1F, 0x7F, 0xFF, 0xFF, 0xF8, 0xE0, 0x00, 0x81, 0x83, 0x83, 0x87, 0x87, 0x87, 0x87, 0x87,
0x87, 0x87, 0x87, 0x87, 0x87, 0xCF, 0xFF, 0xFF, 0xFE, 0x7C, 0x00, 0xC0, 0xF8, 0xFF, 0xFF, 0xFF,
0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x3F, 0xFF, 0xFF, 0xFC, 0xE0, 0x80, 0x00,
0x07, 0x07, 0x27, 0x57, 0x50, 0x90, 0x00, 0xE0, 0x10, 0x10, 0xA0, 0x01, 0x83, 0x47, 0x47, 0xC7,
0x07, 0xF7, 0x27, 0x47, 0xF3, 0x01, 0x00, 0x07, 0x07, 0x07, 0x07, 0xF0, 0x00, 0x00, 0x00, 0x00,
0x00, 0xE0, 0x50, 0x50, 0xE0, 0x00, 0x20, 0x53, 0x57, 0x97, 0x07, 0xF0, 0x57, 0x57, 0x17, 0x07,
0xF7, 0x57, 0xD7, 0x27, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xF7, 0x53, 0xD1, 0x20, 0x06, 0xF7,
0x57, 0x57, 0x17, 0x00, 0xF0, 0x00, 0x00, 0x04, 0x0A, 0x04, 0xE0, 0x50, 0x50, 0xE0, 0x00, 0x37,
0xC7, 0x37, 0x07, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
0x0C, 0x01, 0x0D, 0x01, 0x00, 0x39, 0x44, 0x54, 0x31, 0x00, 0x44, 0x7C, 0x44, 0x00, 0x04, 0x05,
0x7D, 0x05, 0x05, 0x00, 0x00, 0x7D, 0x40, 0x40, 0x41, 0x00, 0x45, 0x7D, 0x45, 0x00, 0x00, 0x7D,
0x09, 0x11, 0x7D, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x7C, 0x10, 0x28, 0x44, 0x00, 0x01, 0x7C,
0x00, 0x01, 0x04, 0x05, 0x7D, 0x05, 0x05, 0x00, 0x0D, 0x01, 0x0D, 0x01, 0x00, 0x00, 0x01, 0x00,
0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 
};

OpticalReceiver phototransistor;  // create an instance of the receiver

void setup()
{
  Serial.begin(9600);                 // start the serial port on the Arduino
  //Serial.print("Size of union-packet: "); Serial.println(sizeof(union packet)); // Debug: check size of array
  phototransistor.set_speed(MODULATE_SPEED);             // laser receive speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  phototransistor.set_rxpin(PHOTOTRANSISTOR_RECEIVE);       // pin the phototransistor is connected to
  phototransistor.set_inverted(true);                       // if receive signal is inverted (Laser on = logic 0) set this to true
  phototransistor.begin();                                  // initialize the receiver

  LCDInit(); //Start the LCD
  /* Cosmetic frivolity on power-up. */
  LCDClear();
  LCDString("Initializing display.");
  delay(750);
  LCDClear();
  LCDBitmap(nasa_worm_BMP);
  delay(2750);
  LCDClear();
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
  displayChanged = false;
  //Serial.print("T: \t");  Serial.print(pkt.temperature); Serial.print("\t H: \t"); Serial.println(pkt.humidity);
  //LCDString("WAITING FOR LASER");
  
  /* block here waiting for a byte to be received */
  byte received_byte;
  bool good_data_read = phototransistor.GetByte(received_byte);
  LCDClear();
  LCDString("LASER RELAY ");
  //LCDCharacter('\n');
  
  if (good_data_read)
  {
    if (valid_read)
    {
      pkt.bytes[receive_count++] = received_byte;
      //Serial.print("Received: \t"); Serial.println(received_byte);

      if (pkt.starter    == '\x01' &&
          pkt.terminator == '\x04')
      {
        // COMPLETE PACKET HAS ARRIVED
        //LCDClear();
        temperature_F = (pkt.temperature * 9/5) + 32; // Convert to awkward imperial units.
        if (pkt.temperature < 1) //hacking in error messages for empty display
        {
          strTemper       = " ERROR `C";
          strTemperFahren = " ERROR `F";          
        }
        else
        {
          strTemper       = "  " + String(pkt.temperature) + "`C   "; //TODO ADD NEWLINES ONCE YOU FIGURE IT OUT
          strTemperFahren = "  " + String(temperature_F) + "`F  ";          
        }
        strHumidi       = "  " + String(pkt.humidity) + "% hum";
        Serial.print("TempC: \t"); Serial.println(pkt.temperature);
        Serial.print("TempF: \t"); Serial.println(temperature_F);
        Serial.print("Humid: \t"); Serial.println(pkt.humidity);
        LCDString(strTemper.c_str());
        LCDString(strTemperFahren.c_str());
        LCDString(strHumidi.c_str());
        delay(2500); // TODO REMOVE THIS LATER -- JUST A TEMP STOP UNTIL I RE-WORK THE TEXT ANIMATION
        reset_packet();
      }
      else if (receive_count > sizeof(union packet) + 1)
      {
        reset_packet();
        //Serial.println(" == Packet's full; reset packet. ==");
      }
    }
    else
    {
      if (received_byte == '\x01')
      // ASCII \x01 is the packet header.
      {
        //Serial.println(" ==     Starting new packet.     ==");
        valid_read = true;
        pkt.bytes[receive_count++] = received_byte;
      }
    }
  }
  // Hacking in a little text animation -- tidy up later if the proof of concept feels solid. -JA
  else
  {
    LCDString("            Waiting for laser"); //hack until I figure out why it won't parse newlines
    //Serial.print(ellipsiscounter);
    //Serial.print("\tWaiting for laser");
    ellipsiscounter++;
    if (ellipsiscounter < (ELLIPSIS_LOOP_DURATION / 4))
    {
      //Serial.println("");
    }
    else if (((ELLIPSIS_LOOP_DURATION / 4) < ellipsiscounter) && (ellipsiscounter < (ELLIPSIS_LOOP_DURATION / 2)))
    {
      LCDString(".");
      //Serial.println(".");
    }
    else if (((ELLIPSIS_LOOP_DURATION / 2) < ellipsiscounter) && (ellipsiscounter < (ELLIPSIS_LOOP_DURATION * 3/4)))
    {
      LCDString("..");
      //Serial.println("..");
    }
    else if (((ELLIPSIS_LOOP_DURATION * 3/4) < ellipsiscounter) && (ellipsiscounter < ELLIPSIS_LOOP_DURATION))
    {
      LCDString("...");
      //Serial.println("...");
    }
    else if (ellipsiscounter >= ELLIPSIS_LOOP_DURATION)
    {
      ellipsiscounter = 0;
    }
  }
} // end main loop

void reset_packet()
{
  valid_read    = false;
  receive_count = 0;
  memset(&pkt, 0, sizeof(union packet));
}
