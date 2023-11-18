//State Diagram
#include "functions.h"
//#include "catclock.h"
#include <EEPROM.h>
//includes for Dynamixel
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

#define LASERCONTROL_PIN PIN_PB0  // output
#define LiDAR_RXD_PIN PIN_PB1     // input
#define PRESSURE_PIN PIN_PB2      // input
#define MOSI_PIN PIN_PB3
#define MISO_PIN PIN_PB4
#define SCK_PIN PIN_PB5
#define MOTOR_DATA_CONTROL PIN_PB6  // output
#define MOTION_IO_PIN PIN_PB7       // input

#define UPBUT_PIN PIN_PC0
#define DOWNBUT_PIN PIN_PC1
#define RIGHTBUT_PIN PIN_PC2
#define LEFTBUT_PIN PIN_PC3
#define MIDBUT_PIN PIN_PC4
#define CLK_PIN PIN_PC5
#define RST_PIN PIN_PC6

#define LED1_PIN PIN_PD0
#define LED2_PIN PIN_PD1
#define CLOCK_CS PIN_PD2
#define SWITCH_PIN PIN_PD3
#define DISPLAY_PIN PIN_PD4  //output LED clock
#define MOTOR_RX PIN_PD5
#define LIDAR_TXD PIN_PD6
#define MOTOR_TX PIN_PD7



#include <TimeLib.h>
#include "LedControl.h"
LedControl lc = LedControl(DISPLAY_PIN, CLK_PIN, CLOCK_CS, 1);
byte dig[] = { 0b10111111, 0b10000110, 0b11011011, 0b11001111, 0b11100110, 0b11101101, 0b11111101, 0b10000111, 0b11111111, 0b11100111 };

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

SoftwareSerial soft_serial(MOTOR_RX, MOTOR_TX);  //UART RX/TX
#define DXL_SERIAL Serial
//#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = MOTOR_DATA_CONTROL;  // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_V = 1;
const uint8_t DXL_H = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setMotor(int phi, int theta) {
  dxl.setGoalCurrent(DXL_H, 200);
  dxl.setGoalPosition(DXL_H, phi, UNIT_DEGREE);
  dxl.setGoalCurrent(DXL_V, 200);
  dxl.setGoalPosition(DXL_V, theta, UNIT_DEGREE);
}

functions f;
//cclock cc(DISPLAY_PIN, CLK_PIN, CLOCK_CS);

#include "TFMini.h"
TFMini tfmini;
SoftwareSerial SerialTFMini(LIDAR_RXD, LIDAR_TXD); 

void getTFminiData(int* distance, int* strength)
{
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (SerialTFMini.available())
  {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59)
    { i = 0;} else if (i == 1 && rx[1] != 0x59){
      i = 0;
    }else if (i == 8){
      for (j = 0; j < 8; j++)
      {checksum += rx[j];}
      if (rx[8] == (checksum % 256))
      {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;} else{i++;}
  }
}
//variables
const float heightOfPost = 0.75;
int length=48;
int i=0;
int j=0;

void setup() {

  Serial.begin(9600);
  pinMode(UPBUT_PIN, INPUT);
  pinMode(DOWNBUT_PIN, INPUT);
  pinMode(RIGHTBUT_PIN, INPUT);
  pinMode(LEFTBUT_PIN, INPUT);
  pinMode(MIDBUT_PIN, INPUT);

  pinMode(SWITCH_PIN, INPUT);

  pinMode(MOTION_IO_PIN, INPUT);  //input from the motion sensor

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(DISPLAY_PIN, OUTPUT);

  //enable internal pullup
  digitalWrite(UPBUT_PIN, HIGH);
  digitalWrite(DOWNBUT_PIN, HIGH);
  digitalWrite(RIGHTBUT_PIN, HIGH);
  digitalWrite(LEFTBUT_PIN, HIGH);
  digitalWrite(MIDBUT_PIN, HIGH);


  // Use UART port of DYNAMIXEL Shield to debug.


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

  //Set real time (Hour)
  int i = 0; //timeconfirm
  int hour[] = {0,0,0};    //ranges from 0 to 23
  int minute[] = {0,0,0};  //ranges from 0 to 59
  //i=0 is current time, i=1 is start time, i = 2 is end time
  showtime(hour[i], minute[i]);

  while (i<3) {
    while (digitalRead(MIDBUT_PIN) != LOW){
    if (digitalRead(UPBUT_PIN) == LOW) {
      hour[i]++;
      if (hour[i] == 24) {
        hour[i] = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      hour[i]--;
      if (hour[i] == -1) {
        hour[i] = 23;
      }
    }
    // updatedisplay
    showtime(hour[i], minute[i]);
    delay(200);
  }
  delay(1000);
  //Set time (Minute)
  while (digitalRead(MIDBUT_PIN) != LOW) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      minute[i]++;
      if (minute[i] == 60) {
        minute[i] = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      minute[i]--;
      if (minute[i] == -1) {
        minute[i] = 59;
      }
    }
    // updatedisplay
    showtime(hour[i], minute[i]);
    delay(200);
  }
  delay(1000);
  if (i==0) setTime(hour[i],minute[i],0,0,0,0);
  i++;
}

  //Turn the laser on
  digitalWrite(LASERCONTROL_PIN, HIGH);
  int confirm = 0;
  int leftlimit = 0;
  int rightlimit = 0;
  int closelimit = 25;
  int farlimit = 25;


  //Calibration State (use buttons Up-Down-Left-Right)

  //Set Left(horizontal motor input phi1)
  //left limit will range from -50 to 0
  //update motor position to x,y = 0,25, middle of array
  int phi = f.find_Phi(0, 2.5) + 90;
  int theta = f.find_Theta(0, 2.5, heightOfPost);

  setMotor(phi, theta);

  //move horizontal motor to initial position

  //move vertical motoe to initial position

  while (confirm == 0) {
    if (digitalRead(LEFTBUT_PIN) == LOW) {
      leftlimit--;
      //Press the left button once and (-1,25)
      phi = f.find_Phi(leftlimit / 10, 2.5);
      //move horizontal motor to phi
    } else if (digitalRead(RIGHTBUT_PIN) == LOW) {
      leftlimit++;
      phi = f.find_Phi(leftlimit / 10, 2.5);
      //move horizontal motor to phi
    } else if (digitalRead(MIDBUT_PIN) == LOW) {
      confirm++;
    }
    //update motor position to leftlimit, 25
    setMotor(phi, theta);
    delay(100);
  }
  //get horizontal data position and set it to phi1
  int p1_phi = phi;

  //Set Rigth(horizontal motor input phi2)
  //right limit will range from 0 to 50
  //smaller phi

  //update motor position to x,y = 0,25
  phi = f.find_Phi(0, 2.5) + 90;
  setMotor(phi, theta);
  //move horizontal motor to initial position
  confirm = 0;
  while (confirm == 0) {
    if (digitalRead(LEFTBUT_PIN) == LOW) {
      rightlimit--;
      phi = f.find_Phi(rightlimit / 10, 2.5) + 90;
      //move horizontal motor to phi
    } else if (digitalRead(RIGHTBUT_PIN) == LOW) {
      rightlimit++;
      phi = f.find_Phi(rightlimit / 10, 2.5) + 90;
      //move horizontal motor to phi
    } else if (digitalRead(MIDBUT_PIN) == LOW) {
      confirm++;
    }
    //update motor position to rightlimit
    setMotor(phi, theta);

    delay(100);
  }
  int p2_phi = phi;
  //Set Close(vertical motor input theta1)

  //update motor position to 0,25
  theta = f.find_Theta(0, 2.5, heightOfPost);
  phi = f.find_Phi(0, 2.5) + 90;
  setMotor(phi, theta);
  //move horizontal motor to initial position
  confirm = 0;
  while (confirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      closelimit--;
      theta = f.find_Theta(0, closelimit / 10, heightOfPost);
      //move vertical motoe to theta
    } else if (digitalRead(DOWNBUT_PIN) == LOW) {
      closelimit++;
      theta = f.find_Theta(0, closelimit / 10, heightOfPost);
      //move vertical motoe to theta
    } else if (digitalRead(MIDBUT_PIN) == LOW) {
      confirm++;
    }
    //update motor position to closelimit
    setMotor(phi, theta);

    delay(100);
  }
  int p1_theta = theta;
  //Set Far(vertical motor input theta2)

  //update motor position to 0,25
  theta = f.find_Theta(0, 2.5, heightOfPost);
  phi = f.find_Phi(0, 2.5) + 90;
  setMotor(phi, theta);

  confirm = 0;
  while (confirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      farlimit--;
      theta = f.find_Theta(0, closelimit / 10, heightOfPost);
    } else if (digitalRead(DOWNBUT_PIN) == LOW) {
      farlimit++;
      theta = f.find_Theta(0, closelimit / 10, heightOfPost);
    } else if (digitalRead(MIDBUT_PIN) == LOW) {
      confirm++;
    }
    //update motor position to 50, farlimit
    setMotor(phi, theta);
    delay(100);
  }
  int p2_theta = theta;
  //update motor position to 50,25 as a final confirmation of
  //completing calibration
  theta = f.find_Theta(0, 2.5, heightOfPost);
  phi = f.find_Phi(0, 2.5) + 90;
  setMotor(phi, theta);

  //turn off laser
  digitalWrite(LASERCONTROL_PIN, LOW);


  //kinematics function here generate array
  int array[48];
  for (int i = 0; i < length * 2; i++) {
    memset(array, 0, sizeof(array));
    f.kinematics(length, array, p1_phi, p1_theta, p2_phi, p2_theta, heightOfPost, i);
    f.writeArrEeprom(length, array, i);//Write rows to Eeprom one by one
  }

  //Lidar
  int read_array[48];
  int distance = 0;
  int strength = 0;
  for (int i= 0; i < length*2; ++i) {
    f.readArrEeprom(length, read_array, i);
    for (int j = 0; j < length; ++j) {
      if (read_array[j] == 1) {
        setMotor(phi,theta);
        getTFminiData(&distance,&strength);
        int diff= abs(f.calcDistance(i-length, j, heightOfPost)-distance);
        if(diff>0.06){
          read_array[j]=0;
        }
      }
    }
    f.writeArrEeprom(length, read_array, i);
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


bool begin=false;
int theta=0;
int phi=0;


void loop() {

   if(digitalRead(MOTION_IO_PIN) == HIGH /*&& Time*/ ){
    begin = true;
  }

  if (digitalRead(MOTION_IO_PIN) == HIGH && begin==true) {
    //Turn the Laser on
    digitalWrite(LASERCONTROL_PIN, HIGH);
    int read_array[48];
    memset(read_array, 0, sizeof(read_array));
    int availablePaths = 0;

    f.readArrEeprom(length, read_array, i - 1);
    if (i > 0 && read_array[j] == 1) {
      availablePaths++;
    }
    f.readArrEeprom(length, read_array, i + 1);
    if (i < length*2 - 1 && read_array[j] == 1) {
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
      if (i < length*2 - 1 && read_array[j] == 1 && randomDirection == 1) {
        i = i + 1;
      }
      f.readArrEeprom(length, read_array, i);
      if (j > 0 && read_array[j - 1] == 1 && randomDirection == 2) {
        j = j - 1;
      }
      if (j < length - 1 && read_array[j + 1] == 1 && randomDirection == 3) {
        j = j + 1;
      }
    }

    //Testing
    //f.readArrEeprom(length, read_array, i);
    //Serial.print(i);
    //Serial.print(", ");
    //Serial.println(j);
    //Serial.println(read_array[j]);


    //Now that the next point has been set, we will move the motors
      theta = f.find_Theta((i - length) / 10.0, j / 10.0, heightOfPost);
      phi = f.find_Phi((i - length) / 10.0, j / 10.0);

    setMotor(phi, theta);
    delay(500);
  }else if (digitalRead(MOTION_IO_PIN)== LOW){
   //If there is no activity by the cat, turn the laser off
    digitalWrite(LASERCONTROL_PIN, LOW);
  }
}
