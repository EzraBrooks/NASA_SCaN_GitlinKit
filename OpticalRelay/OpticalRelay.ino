/***************************************************************************
  Laser Relay Code: Receive signal from a Gitlin laser transmitter and re-
  transmit it to a Gitlin laser receiver / decoder.

  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin 
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication

  Arduino sketch by Tom Gitlin and J. Acevedo
  version 0.0 09 Aug 2018 -- First stab.

***************************************************************************/
#include <HammingEncDec.h>        // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>      // include the modulator/demodulator functionality

const int      linktimeout = 1000;    // if no valid characters received in this time period, assume the link is bad
const int      laserreceivespeed = 2000; //MUST MATCH THE SPEED AS DEFINED IN THE OPTICALTRANSMIT CODE

char           c;                     // variable to hold the byte returned from the receiver
int            i;                     // loop counter
int            measurand;             // tracks the number of measurands (3 for the demo, Temp, Press and Humidity)
unsigned long  timelastchar;          // variable to hold the time the last valid character was received
bool           linkgood;              // flag to store optical link status
bool           timetoswitch;          // flag to cycle through the measurands
bool           blankedvalues;         // flag to determine if values have been blanked for a bad laser link
bool           bit_bucket;            // temporary bucket to hold the inbound bit
unsigned long  timenow;               // holds the Arduino running time in milliseconds for display times
int            LED_transmit_timer_ticker = 0;      //Since we're avoiding using delay(), this will make sure the Xmit LED stays on long enough to be human-noticeable.
                                      
/* IMPORTANT: The Delay function is used sparingly because it stalls program execution
(except for interrupts) and can affects the ability to check if characters are ready. */
                                         
String         parameterValue;      // holds the measurand being built up character-by-character
String         strTemperature, strPressure, strHumidity; // holds the values of the measurands
char           tempChar;
const int      LASERRATE      = 2000;
const int      CHAR_DELAY     = 30;// delay between individual characters of a message (nominally 30ms)
const int      LED_RECEIVE_GOOD    = 6; // Pin value for status lights.
const int      LED_RECEIVE_BAD       = 5; // Pin value for status lights.
const int      LASERPIN       = 13;// Pin to drive laser output
const int      PHOTOT_RECEIVE = 7; // Pin value for the phototransistor input.

char incomingByte;                // variable to hold the byte to be encoded
uint16_t msg;                     // variable to hold the message (character)

OpticalReceiver phototransistor;  // create an instance of the receiver
OpticalTransmitter laser;         // create an instance of the transmitter

void setup()
{
  Serial.begin(9600);                 // start the serial port on the Arduino
  
  
  phototransistor.set_speed(laserreceivespeed);        // laser receive speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  phototransistor.set_rxpin(PHOTOT_RECEIVE);           // pin the phototransistor is connected to
  phototransistor.set_txpin(LASERPIN);      // pin the laser is connected to
  // *** FOR THE MODULES THAT JIMMY BOUGHT, THIS NEEDS TO BE FLIPPED W/R/T TOM'S RECEIVERS (true for a QRD proper, false for bobo versions.)***
  phototransistor.set_inverted(true);                 // if receive signal is inverted (Laser on = logic 0) set this to true
  phototransistor.begin();                             // initialize the receiver
  
  laser.set_speed(LASERRATE);     // laser modulation speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  laser.set_txpin(LASERPIN);      // pin the laser is connected to
  laser.begin();                  // initialize the laser

  // Set up the LED pins to receive output from the Arduino to display status.
  pinMode(LED_RECEIVE_GOOD,  OUTPUT);
  pinMode(LED_RECEIVE_BAD,     OUTPUT);
  
} //END of setup()

// Set up an interrupt service routine to receive characters
// Arduino Timer2 reads the LIGHT_RECEIVE_PIN at laser receive speed to receive each half bit
ISR(TIMER2_COMPA_vect)
{
  //phototransistor.dummy_echo();
  bit_bucket = PIND & (1 << PHOTOT_RECEIVE); //Grab the bit via direct pin access.
  if (bit_bucket)
  {
    digitalWrite(LASERPIN, HIGH);
    timelastchar = millis();
  }
  else
  {
    digitalWrite(LASERPIN, LOW);
  }
}

void loop()
{
  linkgood = !(millis() > (timelastchar + linktimeout));  // update the link status based on the timeout value
  if (linkgood)
  {
    // Power the "GOOD LINK" LED if the link is good. N.b. that the LED is dumb, i.e. the LED
    // will light up even if it's receiving interference/garbage. TODO make this more intelligent. -JA
    digitalWrite(LED_RECEIVE_GOOD, HIGH);
    digitalWrite(LED_RECEIVE_BAD,     LOW);
  }
  else
  {
    // Power the "LINK IS BAD" LED if the link is bad.
    digitalWrite(LED_RECEIVE_GOOD, LOW);
    digitalWrite(LED_RECEIVE_BAD,   HIGH);
  }
} // end main loop()

void  laserTransmit(String xmitmsg)
{
  for (i=0; i<(xmitmsg.length()+1); i++)  // transmit the string byte by byte
  {  
    incomingByte=xmitmsg.charAt(i);       // get the character at position i
    //Serial.print(incomingByte);
    msg = hamming_byte_encoder(incomingByte); // encode the character
    laser.manchester_modulate(msg);       // modulate the character using the laser
    delay(CHAR_DELAY);                    // wait delay between transmitting individual characters of the message
  }
}
