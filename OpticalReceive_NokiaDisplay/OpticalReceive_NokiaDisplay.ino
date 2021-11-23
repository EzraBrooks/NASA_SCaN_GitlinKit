/*TODO:
  * - what happens if ptransistor is wired backwards
  * - [not for launch] more intelligent garbage checking on input; discard and/or replace bad character that are not 0-9, ., or the terminators
 * - re-add text header and comments
 * - [not for launch] drop hundredths place on TX side
 */


#include <Nokia_LCD_Functions.h>  //include library to drive NOKIA display
//N.B. NOKIA 5110 is 5px chars, with 1 pixel padded on either side, for 7 pixels/char * 12 characters for 84 px
#include <HammingEncDec.h>        // include the Hamming encoder/decoder functionality
#include <OpticalModDemod.h>      // include the modulator/demodulator functionality
OpticalReceiver phototransistor;  // create an instance of the receiver
byte           c;                 //holds byte returned from receiver
String         parameterValue;    // holds the measurand being built up character-by-character
String         strTemperatureC, strTemperatureF, strHumidity; // holds the values of the measurands
const uint8_t  NOKIA_SCREEN_MAX_CHAR_WIDTH = 12;
const uint8_t  PIN_PHOTOTRANSISTOR = 2;

int            link_timeout = 1250; // if no valid characters received in this time period, assume the link is bad
unsigned long  time_since_last_character_received = 0; //helps decide when to message about a bad link
unsigned long  time_now = 0; //also helps to decide about bad link
bool           is_the_link_good = true; //boolean for the same bad link decision
const long int ANTISPAM_COUNTER_MAX = 3000;
int            antispam_counter = 0; //An attempt to slow down the spamming of the screen/serial while there isn't a good link. I don't think it's working...
uint8_t        ellipsis_iterator = 0; //iterator to animate a ".  " ".. " "..."

const unsigned char nasa_worm_BMP [] = {
0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x20, 0x20, 0x70, 0x20, 0x20, 0x00, 0x00,
0x02, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0xF0, 0xF8, 0xFC, 0xFE, 0xFE, 0xFE, 0x7E, 0x3C,
0x08, 0xF0, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x08, 0x14, 0x08, 0x00, 0x80, 0x60,
0x80, 0x00, 0x00, 0x20, 0x00, 0x82, 0xC5, 0xE2, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00,
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x01, 0x03, 0x05,
0x05, 0x84, 0xC4, 0xE2, 0xE1, 0xE0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x04, 0x0A, 0x04,
0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,
0xE0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0,
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
0x00, 0xE0, 0x50, 0x50, 0xE0, 0x00, 0x20, 0x53, 0x57, 0x97, 0x07, 0xF7, 0x57, 0x57, 0x17, 0x07,
0xF7, 0x57, 0xD7, 0x27, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xF7, 0x53, 0xD1, 0x20, 0x06, 0xF7,
0x57, 0x57, 0x17, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x50, 0x50, 0xE0, 0x00, 0x37,
0xC7, 0x37, 0x07, 0x06, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
0x00, 0x0D, 0x01, 0x0D, 0x00, 0x01, 0x38, 0x44, 0x55, 0x30, 0x00, 0x44, 0x7C, 0x44, 0x00, 0x05,
0x05, 0x7D, 0x05, 0x04, 0x00, 0x7D, 0x40, 0x40, 0x41, 0x00, 0x45, 0x7D, 0x45, 0x00, 0x00, 0x7D,
0x09, 0x11, 0x7D, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x7C, 0x10, 0x28, 0x44, 0x00, 0x45, 0x7C,
0x44, 0x01, 0x04, 0x05, 0x7D, 0x05, 0x05, 0x00, 0x0D, 0x01, 0x0D, 0x01, 0x00, 0x00, 0x01, 0x00,
0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 
};


void setup() 
{
  Serial.begin(9600);                              // start the serial port on the Arduino
  Serial.println("NASA SCaN Gitlinkit Laser Relay demonstration -- powering on RECEIVER.");
  phototransistor.set_speed(2000);                 // laser receive speed - should be 500+ bits/second, nominal 2000 (=2KHz). Don't mess with this unless you know what you're doing!
  phototransistor.set_rxpin(PIN_PHOTOTRANSISTOR);  // pin the phototransistor is connected to
  phototransistor.set_inverted(true);              // if receive signal is inverted (Laser on = logic 0) set this to true
  phototransistor.begin();                         // initialize the receiver
  
  LCDInit(); //Start the LCD
  /* Cosmetic frivolity on power-up. */
  LCDClear();
  LCDBitmap(nasa_worm_BMP);
  delay(2250);
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
  bool received = phototransistor.GetByte(c);     // get a character from the laser receiver if one is available
  if (received)
  {
   // if a character is ready, look at it
   //Serial.println(c);
   time_since_last_character_received = millis(); //pulls the time elapsed from the Arduino startup in and stores it
   switch (c)
    {       
      // if the character is a terminator, store what was built in a variable and display it
      case 0:
      //Hacking in functionality using LCDCharacter rather than LCDString; it's clunkier but I don't trust LCDString after the last wrestling match I had.
      //This is a kludgy loop to manually /newline align the output.
      //Manually write the 'label' at the front of each line: "T:"
      //Then output the data payload.
      //Then tack on the unit at the end.
      //FIRST, DO THIS FOR CENTIGRADE
        for (int i = 0; i < NOKIA_SCREEN_MAX_CHAR_WIDTH; i++)
        {
          if (i == 0)
          {
            LCDCharacter('T');
          }
          else if (i == 1)
          { 
            LCDCharacter(':');
          }
          else if (i == (strTemperatureC.length() + 3))
          {
            LCDCharacter('C');
          }
          else 
          {
            if (i < (strTemperatureC.length() + 2)) 
            {
              LCDCharacter(strTemperatureC[i - 2]);
            }
            else
            {
              LCDCharacter(' ');
            }
          }
        //END OF THE CENTIGRADE BLOCK
        }
        //THEN, DO THIS FOR FAHRENHEIT
        for (int i = 0; i < NOKIA_SCREEN_MAX_CHAR_WIDTH; i++)
        {
          if (i == 0)
          {
            LCDCharacter('T');
          }
          else if (i == 1)
          { 
            LCDCharacter(':');
          }
          else if (i == (strTemperatureF.length() + 3))
          {
            LCDCharacter('F');
          }
          else 
          {
            if (i < (strTemperatureF.length() + 2)) 
            {
              LCDCharacter(strTemperatureF[i - 2]);
            }
            else
            {
              LCDCharacter(' ');
            }
          }
        //END OF THE FAHRENHEIT BLOCK
        }
        //FINALLY, DO THIS FOR THE HUMIDITY
        for (int i = 0; i < NOKIA_SCREEN_MAX_CHAR_WIDTH; i++)
        {
          if (i == 0)
          {
            LCDCharacter('H');
          }
          else if (i == 1)
          {
            LCDCharacter(':');
          }
          else if (i == (strHumidity.length() + 2))
          {
            LCDCharacter('%');
          }
          else
          {
            if (i < (strHumidity.length() + 2))
            {
              LCDCharacter(strHumidity[i - 2]);
            }
            else
            {
              LCDCharacter(' ');
            }
          }
        //END OF THE HUMIDITY BLOCK
        }
        break;       
      case 70:         // ASCII F termination character for temperature Fahrenheit, use string built to this point for temp
        strTemperatureF=parameterValue;
        parameterValue="";
        Serial.print("[RX] *~ Temp: "); Serial.print(strTemperatureF); Serial.println(" °F ~*");
        break; 
      case 84:         // ASCII T termination character for temperature Centigrade, use string built to this point for temp
        strTemperatureC=parameterValue;
        parameterValue="";
        Serial.print("[RX] *= Temp: "); Serial.print(strTemperatureC); Serial.println(" °C =*");
        break; 
      case 72:        // ASCII H termination character for humidity, use string built to this point for humidity
        strHumidity=parameterValue;
        parameterValue="";
        Serial.print("[RX] *- Hum:  "); Serial.print(strHumidity); Serial.println("%  -*");
        break;
      default :
        parameterValue+=(char)c;  // keep building a string character-by-character until a terminator is found
        break;
    }
  }
  else
  {
    // If we haven't received anything in a while (default: 1250 ms), we presume the laser link has been broken.
    // Let's do some fancy output to complain about the lost laser link in a visually interesting way.
    is_the_link_good = (millis() <= (time_since_last_character_received + link_timeout)); //Check if we're within the timeout parameter
    if (!(is_the_link_good))
    {
      if (antispam_counter >= ANTISPAM_COUNTER_MAX)
      {
        Serial.print(antispam_counter); Serial.print("\t"); Serial.println("Waiting for laser.");
        switch (ellipsis_iterator)
        {
          case 0:
            LCDClear();
            LCDString("No laser    ");
            break;
          case 1:
            LCDString("No laser.   ");
            break;
          case 2:
            LCDString("No laser..  ");
            break;
          case 3:
            LCDString("No laser... ");
            break;
          default:
            ellipsis_iterator = 0;
            break;
        }
        ellipsis_iterator++;
        antispam_counter = 0;
      }
      else
      {
        antispam_counter++;
      }
    }
  }
} // end main loop
