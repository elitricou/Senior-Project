#include <SoftwareSerial.h>                 // importing software serial library  
#include "TFMini.h"                         // importing TFMini.h library
#include "LedControl.h"

TFMini tfmini;                              // creating the object of the class
//Clock pins
#define CLK_PIN PIN_PC5
#define CLOCK_CS PIN_PD2
#define DISPLAY_PIN PIN_PD4

//LiDAR pins
#define LIDAR_RX PIN_PB1
#define LIDAR_TX PIN_PD6

//create TFMini serial
SoftwareSerial SerialTFMini(LIDAR_TX, LIDAR_RX);          //The only value that matters here is the first one

//initialize clock controls
LedControl lc = LedControl(DISPLAY_PIN, CLK_PIN, CLOCK_CS, 1);

//our number library!!
byte dig[] = { 0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111 };
byte alp[] = { 0b01110111, 0b01111111, 0b00111001, 0b01011110, 0b01111001, 0b01110001, 0b00111101, 0b01110110, 0b00110000, 0b00011111, 0b01110101, 0b00111000, 0b00010101, 0b00110111, 0b00111111, 0b01110011, 0b01100111, 0b00110011, 0b01101101, 0b01111000, 0b00111110, 0b00101110, 0b00101010, 0b01110110, 0b01101110, 0b01001011 };

//functions for clock
void showdigit(int pos, int digit) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(dig[digit], i)) lc.setLed(0, i, pos, true);
  }
}
void displayDistance(int distance) {
  lc.clearDisplay(0);
  int first = distance / 1000;
  int second = (distance / 100) % 10;
  int third = (distance / 10) % 10;
  int fourth = distance % 10;
  showdigit(1, first);
  showdigit(2, second);
  showdigit(3, third);
  showdigit(4, fourth);
}
void showletter(int pos, int digit) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(alp[digit], i)) lc.setLed(0, i, pos, true);
  }
}

//functions for LiDAR data collection
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
  //Serial.begin(115200);       //Initialize hardware serial port (serial debug port)
 // while (!Serial);            // wait for serial port to connect. Needed for native USB port only
 // Serial.println ("Initializing...");
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CLOCK_CS, OUTPUT);
  pinMode(DISPLAY_PIN, OUTPUT);
  pinMode(LIDAR_RX, OUTPUT);
  pinMode(LIDAR_TX, INPUT);

  lc.clearDisplay(0);
  showletter(1, 18);   //S
  showletter(2, 19);   //T
  showletter(3, 17);   //R
  showletter(4, 19);   //T
  delay(2000);

  lc.shutdown(0, false);  //to take the LED's out of power-down mode
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
  displayDistance(0);
  delay(200);

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
      displayDistance(distance);                 // printing the distance to the serial port                   
    }
}
