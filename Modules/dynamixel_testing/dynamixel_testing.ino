#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>
#define DXL_SERIAL Serial
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 11;
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
const int DXL_BAUD = 57600;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);
}
void loop() {
  dxl.setGoalCurrent(DXL_ID, 200);
  dxl.setGoalPosition(DXL_ID, 360.0, UNIT_DEGREE);
  delay(50);
  DEBUG_SERIAL.print("Present Current(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID));
  delay(1000);
  dxl.setGoalCurrent(DXL_ID, 300);
  dxl.setGoalPosition(DXL_ID, 0.0, UNIT_DEGREE);
  delay(5000);
}