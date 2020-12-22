/***************************************************************************
  Laser Transmitter Code
  
  This is a program using a BME280 humidity, temperature & pressure sensor
  using I2C to communicate to an Arduino
  
  The sensor is connected to SCL -> SCL, SCA -> SCA, VCC -> 5V, GND -> GND

  A laser is also connected to the Arduino to send data to a receiver

  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin 
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication

  Arduino sketch by Tom Gitlin
  Rewritten to only pass raw data; pushing all formatting to the receiver. -JA
  
***************************************************************************/

#include <Wire.h>             // library to communicate with the sensor using I2C
#include <Adafruit_Sensor.h>  // generic sensor library 
#include <Adafruit_BME280.h>  // BME280 sensor library

#define BME280_ADDRESS (0x76) // Need to set the I2C address to Low = 0x76 (vs High = 0x77)
                              // since the module defaults to 0x77 in the library)

Adafruit_BME280 bme;          // create an I2C instance to talk to the sensor

#include <HammingEncDec.h>    // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>  // include the modulator/demodulator functionality

OpticalTransmitter laser;     // create an instance of the transmitter

unsigned long delaytime=500;  // delay between transmitting measurands in milliseconds (nominally 500ms)
                              // need a 'long' value due to it being compared to the Arduino millis value
int intercharacterdelay=30;   // delay between individual characters of a message (nominally 30ms)
float temperature, pressure, humidity;  // variable to hold values
String strTemperature, strPressure, strHumidity; // String values for the measurands
char incomingByte;            // variable to hold the byte to be encoded
uint16_t msg;                 // varible to hold the message (character)
int LED_SensorError = 5;      // Pin for an LED to indicate something is wrong with the sensor
int i;                        

void setup() 
{
  pinMode(LED_SensorError, OUTPUT);
  Serial.begin(9600);

  // Check for the sensor. If there is an error, blink the SensorError LED
  while (!bme.begin()) 
  {  
    Serial.println(F("Could not find the temp/humidity/pressure sensor. Check the I2C address and/or wiring"));
    digitalWrite(LED_SensorError, HIGH);
    delay (500);
    digitalWrite(LED_SensorError, LOW);
    delay (500);
    //while (1);
  }
  laser.set_speed(2000);      // laser modulation speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  laser.set_txpin(13);        // pin the laser is connected to
  laser.begin();              // initialize the laser
}

// Set up an interrupt service routine to transmit characters
// Arduino Timer2 interrupt toggles the LIGHT_SEND_PIN at the laser send speed to transmit each half bit
ISR(TIMER2_COMPA_vect)
{
  laser.transmit(); // transmit a character if one is ready
}

void loop() 
{
  temperature = bme.readTemperature();  // read the temperature from the sensor. Pemperature is returned in degrees C
  pressure = bme.readPressure()/100;    // read the pressure. Pressure is returned in Pascals, divide by 100 to get millibars
  humidity = bme.readHumidity();        // read the humidity. Humidity is returned in percent relative humidity

  strTemperature=String(temperature)+="T";
  Serial.print(F("Temp: ")); Serial.print(strTemperature);
  laserTransmit(strTemperature);
  
  delay(delaytime);
  
  //strPressure=String(pressure)+="P";
  //Serial.print(F("Pressure: "));
  //Serial.print(strPressure);
  //laserTransmit(strPressure);

  delay(delaytime);
  
  strHumidity=String(humidity)+="H";
  Serial.print(F("Humidity: "));
  Serial.print(strHumidity);
  laserTransmit(strHumidity);
  
  delay(delaytime);
  
  //Serial.println();
}

void  laserTransmit(String xmitmsg)
{
 for (i=0; i<(xmitmsg.length()+1); i++)
 {  // transmit the string byte by byte
    incomingByte=xmitmsg.charAt(i);       // get the character at position i
    //Serial.print(incomingByte);
    msg = hamming_byte_encoder(incomingByte); // encode the character
    laser.manchester_modulate(msg);       // modulate the character using the laser
    delay(intercharacterdelay); // wait delay between transmitting individual characters of the message
  }
}
