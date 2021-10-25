/***************************************************************************
  Laser Transmitter Code
  
  This is a program using an AM2320 humidity and temperature sensor
  using I2C to communicate to an Arduino. A laser is also connected 
  to the Arduino to send data to a receiver.
  
  The sensor is connected to SCL -> SCL, SCA -> SCA, VCC -> 3-5V, GND -> GND.
  DON'T FORGET TO PULLUP YOUR SCL AND SDA LINES TO THE 5V RAIL WITH 2K-10K RESISTORS.
  
  Hamming encoder/decoder and Optical modulator/demodulator are based on
  the LumenWire library written by Andrew Ramanjooloo and modified by
  Tom Gitlin.
  https://github.com/HobbyTransform/Encoded-Laser-and-LED-Serial-Communication

  Arduino sketch by Tom Gitlin with some extra versatility hacked on by Jimmy Acevedo.
  
***************************************************************************/

//=============================================================================================
// INCLUDES
//=============================================================================================

#include <Wire.h>             // library to communicate with the sensor using I2C
#include <Adafruit_Sensor.h>  // generic sensor library 
#include <Adafruit_AM2320.h>  // specific sensor library
#include <HammingEncDec.h>    // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>  // include the modulator/demodulator functionality

//=============================================================================================
// CONSTANTS
//=============================================================================================

const int           CHAR_DELAY     = 30;   // delay between individual characters of a message (nominally 30ms)
const int           PIN_LED_SENSOR = 5;    // Pin for an LED to indicate something is wrong with the sensor
const int           PIN_TX         = 13;   // Pin for the laser transmit wire
const int           MODULATE_SPEED = 2000; // laser modulation speed - should be 500+ bits/second, nominal 2000 (=2KHz)
const unsigned long DELAY_TIME     = 500;  // delay between transmitting measurands in milliseconds (nominally 500ms)

//=============================================================================================
// CLASS INTERFACE
//=============================================================================================

class SensorTransmitterInterface
{
public:
    virtual ~TransmitterInterface(){};
    virtual void read_sensor() = 0;
    virtual void transmit()    = 0;
};

//=============================================================================================
// CLASS
//=============================================================================================

class AdaLaserTransmitter : public SensorTransmitterInterface
{
    //=============================================================================================
    // LOCAL TYPES
    //=============================================================================================
public:
    
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
    
    //=============================================================================================
    // CONSTRUCTORS AND DESTRUCTORS
    //=============================================================================================
public:
    
    AdaLaserTransmitter(Adafruit_AM2320& sensor, OpticalTransmitter& transmitter, int char_delay, unsigned long transmit_delay)
    :   sensor_        (sensor        ),
        transmitter_   (transmitter   ),
        temperature_   (0.0f          ),
        humidity_      (0.0f          ),
        char_delay_    (char_delay    ),
        transmit_delay_(transmit_delay)
        
    {};

    //=============================================================================================
    // CLASS API
    //=============================================================================================
public:
    virtual void read_sensor()
    {
        temperature_ = sensor_.readTemperature();
        Serial.print("Temp: \t" + temperature_);
        humidity_    = sensor_.readHumidity();
        Serial.print("Humi: \t" + humidity_);
    }
    
    virtual void transmit()
    {
        union packet pkt = build_packet();
        
        // Send the Packet
        for (int i = 0; i < sizeof(union packet); i++)
        {
            uint16_t encoded_msg = hamming_byte_encoder(pkt.bytes[i]);
            transmitter_.manchester_modulate(encoded_msg);       // modulate the character using the laser
            delay(char_delay_);
        }
        
        // Waiting for the whole packet to go out (not sure this is needed)
        delay(transmit_delay_);
    };
    
    //=============================================================================================
    // INTERNAL OPERATIONS
    //=============================================================================================
private:
    
    union packet build_packet()
    {
        union packet pkt;
        pkt.starter     = '\x01';
        pkt.temperature = temperature_;
        pkt.humidity    = humidity_;
        pkt.terminator  = '\x04';
        
        return pkt;
    };
    
    //=============================================================================================
    // INTERNAL DATA
    //=============================================================================================
private:
    
    Adafruit_AM2320&    sensor_;
    OpticalTransmitter& transmitter_;
    float               temperature_;
    float               humidity_;
    const int           char_delay_;
    const unsigned long transmit_delay_;
};

//=============================================================================================
// GLOBALS
//=============================================================================================

Adafruit_AM2320             am2320();    // create an I2C instance to talk to the sensor
OpticalTransmitter          laser;       // create an instance of the transmitter
SensorTransmitterInterface* transmitter;

/* --------- ACTUAL SETUP ---------- */
void setup() 
{
    //=============================================================================================
    // Hardware Setup
    //=============================================================================================
    
    pinMode(PIN_LED_SENSOR, OUTPUT);
    Serial.begin(9600);
    Serial.println("==================================================");
    Serial.println("NASA SCaN 'GitlinKit' Laser Relay - Transmitter");
    Serial.println("Powering on...");
    Serial.println("==================================================");
  
    //=============================================================================================
    // Initialize our dependencies
    //=============================================================================================
  
    // AM2320
    am2320.begin();
    
    // Check for the sensor. If there is an error, blink the SensorError LED
    while (!am2320.begin())
    {
        Serial.println(F("Could not find the temp/humidity/pressure sensor. Check the I2C address and/or wiring"));
        digitalWrite(PIN_LED_SENSOR, HIGH);
        delay (500);
        digitalWrite(PIN_LED_SENSOR, LOW);
        delay (500);
    }
  
    // Optical Transmitter
    laser.set_speed(MODULATE_SPEED);
    laser.set_txpin(PIN_TX);
    laser.begin();              // initialize the laser
  
    // Interface
    //  We can use polymorphism here to use a interface but give it AdaLaser specific data.
    //  The benefit here is because we are using a standard interface "class SensorTransmitterInterface"
    //  We could plug anything into it as long as it inherits from and uses the two methods "read_sensor()" and
    //  "transmit()". Thus our main loop never changes, just what we plug in. We could have a BmpLaserTransmitter
    //  class that takes a bmp instead of a am2320 and the only thing that would change is this line just below.
    //  Ideally, the classes above would be kept in their own .h files as well to clean up the program.
    transmitter = new AdaLaserTransmitter(am2320, laser, CHAR_DELAY, delaytime)

} /* END of setup(); */

/* --------- INTERRUPT SERVICE ROUTINE (ISR) ----------
 *  Set up an Interrupt Service Routine to transmit characters.
 * Arduino Timer2 interrupt toggles the LIGHT_SEND_PIN at the 
 * laser send speed to transmit each half bit. */
ISR(TIMER2_COMPA_vect)
{
    laser.transmit(); // transmit a character if one is ready
}

/* --------- MAIN ---------- */
void loop() 
{
    transmitter->read_sensor();
    transmitter->transmit();
} /*END of loop() */
