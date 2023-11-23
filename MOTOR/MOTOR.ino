#include <Dynamixel2Arduino.h>

#include <SoftwareSerial.h>
#include "functions.h"

#define LASER_PIN 4
SoftwareSerial soft_serial(7, 8);  //UART RX/TX
#define DXL_SERIAL Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_V = 2;
const uint8_t DXL_H = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

functions f;

void setMotor(int phi, int theta) {
  phi=phi+136.4;
  if (phi < 315 && phi > 136.4) {
    dxl.setGoalCurrent(DXL_H, 60);
    dxl.setGoalPosition(DXL_H, phi, UNIT_DEGREE);
  }
  theta=theta+100;
  if (theta < 145 && theta > 100) {
    dxl.setGoalCurrent(DXL_V, 60);
    dxl.setGoalPosition(DXL_V, theta, UNIT_DEGREE);
  }
}

const float heightOfPost = 0.75;
int length = 48;
int i = 0;
int j = 0;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(9600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_V);
  dxl.ping(DXL_H);
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_V);
  dxl.torqueOff(DXL_H);
  dxl.setOperatingMode(DXL_V, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(DXL_H, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_V);
  dxl.torqueOn(DXL_H);

  //Laser
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);  //Turn the laser off

  //kinematics
  int p1_phi = 50;
  int p2_phi = 100;
  int p1_theta = 50;
  int p2_theta = 15;

  int array[48];
  for (int i = 0; i < length * 2; i++) {
    memset(array, 0, sizeof(array));
    f.kinematics(length, array, p1_phi, p1_theta, p2_phi, p2_theta, heightOfPost, i);
    f.writeArrEeprom(length, array, i);
  }

  //Set random initial point(from the ones)
  randomSeed(analogRead(0));
  i = random(0, 96);
  j = random(0, 48);
  f.readArrEeprom(length, array, i);
  while (array[j] != 1) {
    i = random(0, 96);
    j = random(0, 48);
    f.readArrEeprom(length, array, i);
  }
}

bool begin = false;
int theta = 0;
int phi = 0;
void loop() {
  // put your main code here, to run repeatedly:

  // Set Goal Cuint
  int iv_present_position = 0;
  int ih_present_position = 0;
  float fv_present_position = 0.0;
  float fh_present_position = 0.0;

  // put your main code here, to run repeatedly:
  fv_present_position = dxl.getPresentPosition(DXL_V, UNIT_DEGREE);
  fh_present_position = dxl.getPresentPosition(DXL_H, UNIT_DEGREE);

  

  //Turn the Laser on
  digitalWrite(LASER_PIN, HIGH);

  int read_array[48];
  memset(read_array, 0, sizeof(read_array));
  int availablePaths = 0;

  f.readArrEeprom(length, read_array, i - 1);
  if (i > 0 && read_array[j] == 1) {
    availablePaths++;
  }
  f.readArrEeprom(length, read_array, i + 1);
  if (i < length * 2 - 1 && read_array[j] == 1) {
    availablePaths++;
  }
  f.readArrEeprom(length, read_array, i);
  if (j > 0 && read_array[j - 1] == 1) {
    availablePaths++;
  }
  if (j < length - 1 && read_array[j + 1] == 1) {
    availablePaths++;
  }

  if (availablePaths > 0) {
    // Randomly select one of the available paths
    int randomDirection = random(0, availablePaths);

    f.readArrEeprom(length, read_array, i - 1);
    if (i > 0 && read_array[j] == 1 && randomDirection == 0) {
      i = i - 1;
    }
    f.readArrEeprom(length, read_array, i + 1);
    if (i < length * 2 - 1 && read_array[j] == 1 && randomDirection == 1) {
      i = i + 1;
    }
    f.readArrEeprom(length, read_array, i);
    if (j > 0 && read_array[j - 1] == 1 && randomDirection == 2) {
      j = j - 1;
    }
    if (j < length - 1 && read_array[j + 1] == 1 && randomDirection == 3) {
      j = j + 1;
    }

    //Testing
    //f.readArrEeprom(length, read_array, i);
    //Serial.print(i);
    //Serial.print(", ");
    //Serial.println(j);
    //Serial.println(read_array[j]);


    //Now that the next point has been set, we will move the motors
    theta = f.find_Theta((i - length) / 10.0, j / 10.0, heightOfPost);
    phi = 90 - f.find_Phi((i - length) / 10.0, j / 10.0);

    setMotor(phi, theta);
    delay(50);
  }
}
