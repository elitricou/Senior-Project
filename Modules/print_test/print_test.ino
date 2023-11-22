#include <Dynamixel.h>

Dynamixel dxl(11);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(2000);
  //Serial.print("Test");
  dxl.attach(Serial, 57600);
  dxl.torqueEnable(1,true);
  dxl.goalPosition(1400);
}

void loop() {
  // put your main code here, to run repeatedly:

}
