/***************************************************************************
  Laser Transmitter Code
  
  This program uses an Adafruit AM2320 humidity & temperature sensor to 
  communicate to an Arduino over I2C. The Arduino then packetizes the data
  and sends it to a second Arduino via laser link, through a KY-008 laser
  module.

  This is intended to pair with the [thus far] unofficial NASA Space Communications
  and Navigation "GitlinKit" instruction packet. Inquire at 
  GSFC-SCaN-ENGAGEMENT@MAIL.NASA.GOV for a copy of these instructions.
  
  The sensor is connected to SCL -> SCL, SCA -> SCA, VCC -> 5V, GND -> GND

  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin.
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication
  Original Arduino sketch, prototyping, and concept by Tom Gitlin.
  Modifications to refactor the code, use the Nokia 5110 display, and a 'no
  solder' version by Jimmy Acevedo and Julie Hoover. 
  
  Thanks to Dan Koris and Ezra Brooks for QA and debugging.
  
***************************************************************************/

#include <Wire.h>             // library to communicate with the sensor using I2C
#include <Adafruit_Sensor.h>  // generic sensor library 
#include <Adafruit_AM2320.h>  // specific sensor library

Adafruit_AM2320 am2320 = Adafruit_AM2320(); // create an I2C instance to talk to the sensor

#include <HammingEncDec.h>    // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>  // include the modulator/demodulator functionality

OpticalTransmitter laser;     // create an instance of the transmitter

unsigned long       delaytime     = 500;  // delay between transmitting measurands in milliseconds (nominally 500ms)
// need a 'long' value due to it being compared to the Arduino millis value
const int           CHAR_DELAY    = 30;   // delay between individual characters of a message (nominally 30ms)
float               temperatureC, temperatureF, humidity = 0;      // Float values for the measurands
String              strTemperatureC, strTemperatureF, strHumidity; // String values for the measurands
char                incomingByte;                                  // variable to hold the byte to be encoded
uint16_t            msg;                                           // variable to hold the message (character)
const uint8_t       PIN_LASER_XMIT = 13;                           //Pin for the laser transmitter
bool                sensor_good = false;  //Variable to check if the AAM2320 sensor is reading properly.

void setup()
{
  Serial.begin(9600);
  Serial.println("NASA SCaN Gitlinkit Laser Relay demonstration -- powering on TRANSMITTER.");
  laser.set_speed(2000);      // laser modulation speed - should be 500+ bits/second, nominal 2000 (=2KHz). Don't change this!
  laser.set_txpin(PIN_LASER_XMIT);        // pin the laser is connected to
  laser.begin();              // initialize the laser
  am2320.begin();
  temperatureC = am2320.readTemperature();
  //If the AM2320 isn't set up properly, then float temperatureC will read as NaN (not a number) and won't resolve as == itself
  if (temperatureC == temperatureC)
  {
    Serial.print("AM2320 sensor has been set up properly! Initial measurement is temperature: "); Serial.print(am2320.readTemperature()); Serial.println(" C.");
  }
  else
  {
    Serial.println("Not reading the AM2320 sensor; check the wiring and assembly instructions carefully.");
  }
} // END of setup();

/* Set up an Interrupt Service Routine to transmit characters.
   Arduino Timer2 interrupt toggles the LIGHT_SEND_PIN at the
   laser send speed to transmit each half bit. */
ISR(TIMER2_COMPA_vect)
{
  laser.transmit(); // transmit a character if one is ready
}

void loop()
{
  temperatureC = am2320.readTemperature();  // read the temperature from the sensor. [degrees C]
  temperatureF = temperatureC * 1.8 + 32.0; // convert C to F
  humidity     = am2320.readHumidity();     // read the humidity. Humidity is returned in percent relative humidity

  //If the AM2320 isn't set up properly, then float temperatureC will read as NaN (not a number) and won't resolve as == itself
  if (!(temperatureC == temperatureC))
  {
    Serial.println("Not reading the AM2320 sensor; check the wiring and assembly instructions carefully.");
  }

  strTemperatureC = String(temperatureC) += "T";
  Serial.print("[TX] Temp: "); Serial.print(strTemperatureC); Serial.println(" °C");
  laserTransmit(strTemperatureC);
  delay(delaytime);
  strTemperatureF = String(temperatureF) += "F";
  Serial.print("[TX] Temp: "); Serial.print(strTemperatureF); Serial.println(" °F ");
  laserTransmit(strTemperatureF);
  delay(delaytime);

  strHumidity = String(humidity) += "H";
  Serial.print("[TX] Hum:  "); Serial.print(strHumidity); Serial.println("% ");
  laserTransmit(strHumidity);
  delay(delaytime);

} //END of loop()

void  laserTransmit(String xmitmsg)
{
  for (int i = 0; i < (xmitmsg.length() + 1); i++) // transmit the string byte by byte
  {
    incomingByte = xmitmsg.charAt(i);     // get the character at position i
    //Serial.print(incomingByte);
    msg = hamming_byte_encoder(incomingByte); // encode the character
    laser.manchester_modulate(msg);       // modulate the character using the laser
    delay(CHAR_DELAY);                    // wait delay between transmitting individual characters of the message
  }
} // END of laserTransmit()
