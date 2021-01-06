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

const unsigned char nasa_worm_BMP [] = {
0x00, 0xFC, 0xFE, 0xFF, 0xFF, 0x07, 0x0F, 0x1F, 0xFE, 0xFC, 0xF8, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0xFE, 0xFE, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFE, 0xFE, 0x1F, 0x0F, 0x0F,
0x7E, 0xFE, 0xFC, 0xF0, 0x80, 0x00, 0x00, 0xE0, 0xF8, 0xFC, 0xFE, 0x9E, 0x0E, 0x0E, 0x0E, 0x0E,
0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xFC, 0xFE,
0x7E, 0x1E, 0x0E, 0x1E, 0xFE, 0xFC, 0xF8, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
0xFF, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x7F, 0xFF, 0xFC, 0xE0, 0x80, 0x00, 0x80, 0xFF, 0xFF, 0xFF,
0xFF, 0x00, 0xC0, 0xF8, 0xFF, 0xFF, 0x3F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0xFF,
0xFF, 0xFE, 0xF0, 0x80, 0x03, 0x07, 0x07, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
0x9F, 0xFE, 0xFE, 0xFC, 0x70, 0x80, 0xF0, 0xFE, 0xFF, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00,
0x00, 0x07, 0x3F, 0xFF, 0xFE, 0xF0, 0xC0, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x01, 0x06, 0x07, 0x07,
0x07, 0x01, 0x00, 0x00, 
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
  LCDBitmap(nasa_worm_BMP);
  delay(1000);
  LCDString("NASA SCaN Laser Relay Kit\n");
  LCDString("Initializing display...");
  delay(750);
  LCDClear();
  LCDString("Initial design by Tom Gitlin. Thanks, Tom!");
  delay(200);
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
        Serial.print("Temp: \t"); Serial.println(pkt.temperature);
        Serial.print("Humi: \t"); Serial.println(pkt.humidity);
        LCDClear();
        temperature_F = (pkt.temperature * 9/5) + 32; // Convert to awkward imperial units.
        strTemper       = "Temp: " + String(pkt.temperature) + " `C" + '\n';
        strTemperFahren = "      " + String(temperature_F) + " `F" + '\n';
        strHumidi       = "Humi: " + String(pkt.humidity) + "%";
        LCDString(strTemper.c_str());
        LCDString(strHumidi.c_str());
        reset_packet();
      }
      else if (receive_count > sizeof(union packet) + 1)
      {
        reset_packet();
        Serial.println(" == Packet's full; reset packet. ==");
      }
    }
    else
    {
      if (received_byte == '\x01')
      // ASCII \x01 is the packet header.
      {
        Serial.println(" ==     Starting new packet.     ==");
        valid_read = true;
        pkt.bytes[receive_count++] = received_byte;
      }
    }
  }
} // end main loop

void reset_packet()
{
  valid_read    = false;
  receive_count = 0;
  memset(&pkt, 0, sizeof(union packet));
}
