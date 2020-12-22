
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
 
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
 
 
float AccelX = 0;
float AccelY = 0;
float AccelZ = 0;

 
 
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("ADXL345 Accelerometer Calibration"); 
  Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  
  Serial.println("Type key when ready..."); 
    while (!Serial.available()){}  // wait for a character
}
 
void loop(void)
{
    
    
    /* Get a new sensor event */ 
    sensors_event_t accelEvent;  
    accel.getEvent(&accelEvent);
    
    AccelX = accelEvent.acceleration.x;
   
    AccelY = accelEvent.acceleration.y;

    AccelZ = accelEvent.acceleration.z;
  
    Serial.print("Accel X, Y, Z: "); Serial.print(AccelX); Serial.print("  ");Serial.print(AccelY); Serial.print("  "); Serial.print(AccelZ); Serial.println();
    delay(300);
 
    while (Serial.available())
    {
      Serial.read();  // clear the input buffer
    }
}
