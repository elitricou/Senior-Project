#include <Dynamixel2Arduino.h>

#include <SoftwareSerial.h>

#include <TimeLib.h>
#include "LedControl.h"
// Clock LED by Elisabeth Tricou


//Clock pins
#define CLK_PIN PIN_PC5
#define CLOCK_CS PIN_PD2
#define DISPLAY_PIN PIN_PD4


#define LIDAR_RX PIN_PB1
#define LIDAR_TX PIN_PD6

//#define MOTOR_RX 5
//#define MOTOR_TX 7
#define DXL_DIR_PIN PIN_PB6

#define unused1 PIN_PD5
#define unused2 PIN_PD7


SoftwareSerial soft_serial(LIDAR_RX, LIDAR_TX); //UART RX/TX
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
//const int DXL_DIR_PIN = 20; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_V = 1;
const uint8_t DXL_H = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

LedControl lc = LedControl(DISPLAY_PIN, CLK_PIN, CLOCK_CS, 1);
byte dig[] = { 0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111 };
byte alp[] = { 0b01110111, 0b01111111, 0b00111001, 0b01011110, 0b01111001, 0b01110001, 0b00111101, 0b01110110, 0b00110000, 0b00011111, 0b01110101, 0b00111000, 0b00010101, 0b00110111, 0b00111111, 0b01110011, 0b01100111, 0b00110011, 0b01101101, 0b01111000, 0b00111110, 0b00101110, 0b00101010, 0b01110110, 0b01101110, 0b01001011 };

void showletter(int pos, int digit) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(alp[digit], i)) lc.setLed(0, i, pos, true);
  }
}
void showdigit(int pos, int digit) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(dig[digit], i)) lc.setLed(0, i, pos, true);
  }
}
void showtime(int hours, int mins) {
  lc.clearDisplay(0);
  int first = hours / 10;
  int second = hours % 10;
  int third = mins / 10;
  int fourth = mins % 10;
  showdigit(1, first);
  showdigit(2, second);
  showdigit(3, third);
  showdigit(4, fourth);
}
int h = 0;
int m = 0;
int s_h = 0;
int s_m = 0;
int e_h = 0;
int e_m = 0;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CLOCK_CS, OUTPUT);
  pinMode(DISPLAY_PIN, OUTPUT);
  pinMode(DXL_DIR_PIN, OUTPUT);
  pinMode(unused1, INPUT);
  pinMode(unused2, INPUT);


  int8_t found_dynamixel = 0;


  lc.shutdown(0, false);  //to take the LED's out of power-down mode
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
  showtime(0, 0);
  delay(200);
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(9600);
  delay(300);

  if (DEBUG_SERIAL.isListening()){
    DEBUG_SERIAL.print("It works!");
  }
  else{
    lc.clearDisplay(0);
    showdigit(1, 3);  //R
    showdigit(2, 3);   //E
    showdigit(3, 3);   //A
    showdigit(4, 3);  //L
  }



  lc.clearDisplay(0);
  showletter(1, 17);  //R
  showletter(2, 4);   //E
  showletter(3, 0);   //A
  showletter(4, 11);  //L

  delay(2000);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);

  for(int id = 0; id<3 ; id++){
    if(dxl.ping(id)){
      DEBUG_SERIAL.print("ID : ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.print(", Model Number: ");
      DEBUG_SERIAL.println(dxl.getModelNumber(id));
            found_dynamixel++;}
  }

  DEBUG_SERIAL.print("Total ");
  DEBUG_SERIAL.print(found_dynamixel);
  DEBUG_SERIAL.println(" DYNAMIXEL(s) found!");
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_V);
  dxl.ping(DXL_H);
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_V);
  dxl.torqueOff(DXL_H);
  dxl.setOperatingMode(DXL_V, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(DXL_H, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_V);
  dxl.torqueOn(DXL_H);
}

void loop() {
  // put your main code here, to run repeatedly:
   
  lc.clearDisplay(0);
  showletter(1, 18);   //S
  showletter(2, 19);   //T
  showletter(3, 17);   //R
  showletter(4, 19);   //T

  // Set Goal Current using RAW value
  dxl.setGoalCurrent(DXL_H, 200);
  dxl.setGoalPosition(DXL_H, 350.0, UNIT_DEGREE);
  dxl.setGoalCurrent(DXL_V, 4.0, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_V, 0);
  delay(500);
  dxl.setGoalCurrent(DXL_V, 150);
  dxl.setGoalPosition(DXL_V, 300.0, UNIT_DEGREE);
  // Print present current
  delay(500);
  //DEBUG_SERIAL.print("Present Current H(raw) : ");
  //DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_H));
  
  delay(5000);

  // Set Goal Current 3.0% using percentage (-100.0 [%] ~ 100.0[%])
  dxl.setGoalCurrent(DXL_H, 3.0, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_H, 0);
  dxl.setGoalCurrent(DXL_V, 250);
  dxl.setGoalPosition(DXL_V, 350.0, UNIT_DEGREE);
  
  // Print present current in percentage
  delay(500);
  //DEBUG_SERIAL.print("Present Current H(ratio) : ");
 // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_H, UNIT_PERCENT));
  
  lc.clearDisplay(0);
  showletter(2, 4);   //E
  showletter(3, 13);   //N
  showletter(4, 3);   //D

  delay(5000);
}
