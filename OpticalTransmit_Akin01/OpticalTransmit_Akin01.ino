#include <Wire.h>             // library to communicate with the sensor using I2C
#include <Adafruit_Sensor.h>  // generic sensor library 
#include <Adafruit_AM2320.h>  // specific sensor library

/*#define BME280_ADDRESS (0x76) // Need to set the I2C address to Low = 0x76 (vs High = 0x77)
  */
Adafruit_AM2320 am2320 = Adafruit_AM2320(); // create an I2C instance to talk to the sensor

#include <HammingEncDec.h>    // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>  // include the modulator/demodulator functionality

OpticalTransmitter laser;     // create an instance of the transmitter

unsigned long delaytime = 500;  // delay between transmitting measurands in milliseconds (nominally 500ms)
                              // need a 'long' value due to it being compared to the Arduino millis value
const int CHAR_DELAY    = 30;   // delay between individual characters of a message (nominally 30ms)
float temperature, humidity = 0; 
boolean english=false;         // flag to use english units, otherwise use SI
String strTemperature, strHumidity; // String values for the measurands
char incomingByte;            // variable to hold the byte to be encoded
uint16_t msg;                 // varible to hold the message (character)
int i;                        

void setup() 
{
  Serial.begin(9600);
  am2320.begin();
  laser.set_speed(2000);      // laser modulation speed - should be 500+ bits/second, nominal 2000 (=2KHz)
  laser.set_txpin(13);        // pin the laser is connected to
  laser.begin();              // initialize the laser

} // END of setup();

/* Set up an Interrupt Service Routine to transmit characters.
 * Arduino Timer2 interrupt toggles the LIGHT_SEND_PIN at the 
 * laser send speed to transmit each half bit. */
ISR(TIMER2_COMPA_vect)
{
  laser.transmit(); // transmit a character if one is ready
}

void loop() 
{
    temperature = am2320.readTemperature();  // read the temperature from the sensor. [degrees C]
    humidity    = am2320.readHumidity();     // read the humidity. Humidity is returned in percent relative humidity

    // if english units are desired, perform the conversion
    if (english) 
    {
      temperature = temperature * 1.8 + 32.0; // convert C to F
    }
 
    strTemperature=String(temperature)+="T";
    Serial.println(strTemperature);
    laserTransmit(strTemperature);
    delay(delaytime);
    
    strHumidity=String(humidity)+="H";
    Serial.println(strHumidity);
    laserTransmit(strHumidity);
    delay(delaytime);

} //END of loop()

void  laserTransmit(String xmitmsg)
{
   for (i=0; i<(xmitmsg.length()+1); i++)   // transmit the string byte by byte
   {
      incomingByte=xmitmsg.charAt(i);       // get the character at position i
      //Serial.print(incomingByte);
      msg = hamming_byte_encoder(incomingByte); // encode the character
      laser.manchester_modulate(msg);       // modulate the character using the laser
      delay(CHAR_DELAY);                    // wait delay between transmitting individual characters of the message
    }
} // END of laserTransmit()
