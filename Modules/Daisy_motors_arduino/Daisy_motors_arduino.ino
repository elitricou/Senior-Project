#include <Dynamixel2Arduino.h>

#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); //UART RX/TX
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_V = 2;
const uint8_t DXL_H = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(9600);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
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
  int iv_present_position = 0;
  int ih_present_position = 0;
  float fv_present_position = 0.0;
  float fh_present_position = 0.0;

  // put your main code here, to run repeatedly:
  fv_present_position = dxl.getPresentPosition(DXL_H, UNIT_DEGREE);
  DEBUG_SERIAL.print("Present_Vertical_Position(degree) : ");
  DEBUG_SERIAL.println(fv_present_position); 

  iv_present_position = dxl.getPresentPosition(DXL_H);
  DEBUG_SERIAL.print("Present_Vertical_Position(raw) : ");
  DEBUG_SERIAL.println(iv_present_position);

  delay(1000);
  // Set Goal Current using RAW value
  dxl.setGoalCurrent(DXL_H, 50);
  dxl.setGoalPosition(DXL_H, 219.0, UNIT_DEGREE);
  dxl.setGoalCurrent(DXL_V, 4.0, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_V, 120);
  delay(500);
  dxl.setGoalCurrent(DXL_V, 25);
  dxl.setGoalPosition(DXL_V, 100.0, UNIT_DEGREE);
  // Print present current
  delay(500);
  DEBUG_SERIAL.print("Present Current H(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_H));
  
  delay(5000);

  // Set Goal Current 3.0% using percentage (-100.0 [%] ~ 100.0[%])
  /*dxl.setGoalCurrent(DXL_H, 3.0, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_H, 0);
  dxl.setGoalCurrent(DXL_V, 100);
  dxl.setGoalPosition(DXL_V, 30.0, UNIT_DEGREE);
  
  // Print present current in percentage
  delay(500);
  DEBUG_SERIAL.print("Present Current H(ratio) : ");
  DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_H, UNIT_PERCENT));
  
  delay(5000);*/
}