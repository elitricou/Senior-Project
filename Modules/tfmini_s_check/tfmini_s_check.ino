#include <SoftwareSerial.h>                 // importing software serial library  
#include "TFMini.h"                         // importing TFMini.h library
TFMini tfmini;                              // creating the object of the class
 
SoftwareSerial SerialTFMini(3, 2);          //The only value that matters here is the first one, 2, Rx
 
void getTFminiData(int* distance, int* strength)
{
// some calculations related to the sensor data 
// check the datasheet to understand it
  
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (SerialTFMini.available())
  {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59)
    {
      i = 0;
    }
    else if (i == 1 && rx[1] != 0x59)
    {
      i = 0;
    }
    else if (i == 8)
    {
      for (j = 0; j < 8; j++)
      {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256))
      {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    }
    else
    {
      i++;
    }
  }
}
 
 
void setup()
{
  Serial.begin(115200);       //Initialize hardware serial port (serial debug port)
  while (!Serial);            // wait for serial port to connect. Needed for native USB port only
  Serial.println ("Initializing...");
  SerialTFMini.begin(TFMINI_BAUDRATE);    //Initialize the data rate for the SoftwareSerial port
  tfmini.begin(&SerialTFMini);            //Initialize the TF Mini sensor
}
 
void loop()
{
  int distance = 0;
  int strength = 0;
 
  getTFminiData(&distance, &strength);          // function for getting the data 
 
    getTFminiData(&distance, &strength);
    if (distance)
    {
      Serial.print(distance);                 // printing the distance to the serial port
      Serial.println("cm\t");                   
    }
}
