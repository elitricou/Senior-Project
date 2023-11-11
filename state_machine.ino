//State Diagram
#include "functions.h"
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
LedControl lc = LedControl(11, 13, 10, 1);
byte dig[] = { 0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01100111 };

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

void showdigit(int pos, int digit, bool point) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(dig[digit], i)) lc.setLed(0, i, pos, true);
  }
  if (point) lc.setLed(0, 7, pos, true);
}

void showtime(int hours, int mins) {
  lc.clearDisplay(0);

  int first = hours / 10;
  int second = hours % 10;
  int third = mins / 10;
  int fourth = mins % 10;
  showdigit(1, first, false);
  showdigit(2, second, true);
  showdigit(3, third, true);
  showdigit(4, fourth, false);
}


functions f;
const float heightOfPost = 0.75;

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



  //At any time switch is turned OFF turn the device off
  //skip...

  //Set acceptable input for each Time set, ...

  //Turn On the Device when switch is turned on
  //skip...

  //Input Up Down button

  //Set real time (Hour)
  int hourconfirm = 0;
  int minuteconfirm = 0;
  int hour = 0;    //ranges from 0 to 23
  int minute = 0;  //ranges from 0 to 59

  showtime(hour, minute);

  while (hourconfirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      hour++;
      if (hour == 24) {
        hour = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      hour--;
      if (hour == -1) {
        hour = 23;
      }
    }
    if (digitalRead(MIDBUT_PIN) == LOW) {
      hourconfirm++;
    }
    // updatedisplay
    showtime(hour, minute);

    delay(200);
  }

  //Set time (Minute)
  while (minuteconfirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      minute++;
      if (minute == 60) {
        minute = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      minute--;
      if (minute == -1) {
        minute = 59;
      }
    }
    if (digitalRead(MIDBUT_PIN) == LOW) {
      minuteconfirm++;
    }
    // updatedisplay
    showtime(hour, minute);

    delay(200);
  }

  //Set time start (Hour)
  hourconfirm = 0;
  minuteconfirm = 0;

  int starthour = 0;
  int startminute = 0;
  while (hourconfirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      starthour++;
      if (starthour == 24) {
        starthour = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      starthour--;
      if (hour == -1) {
        hour = 23;
      }
    }
    if (digitalRead(MIDBUT_PIN) == LOW) {
      hourconfirm++;
    }
    // updatedisplay
    showtime(hour, minute);

    delay(200);
  }

  //Set time start (Minute)
  while (minuteconfirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      startminute++;
      if (startminute == 60) {
        startminute = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      startminute--;
      if (startminute == -1) {
        startminute = 59;
      }
    }
    if (digitalRead(MIDBUT_PIN) == LOW) {
      minuteconfirm++;
    }
    // updatedisplay
    showtime(hour, minute);

    delay(200);
  }

  //Set time end (Hour)
  //Set time end (Minute)
  hourconfirm = 0;
  minuteconfirm = 0;

  int endhour = 0;
  int endminute = 0;
  while (hourconfirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      endhour++;
      if (endhour == 24) {
        endhour = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      endhour--;
      if (endhour == -1) {
        endhour = 23;
      }
    }
    if (digitalRead(MIDBUT_PIN) == LOW) {
      hourconfirm++;
    }
    // updatedisplay
    showtime(hour, minute);

    delay(200);
  }
  while (minuteconfirm == 0) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      endminute++;
      if (endminute == 60) {
        endminute = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      endminute--;
      if (endminute == -1) {
        endminute = 59;
      }
    }
    if (digitalRead(MIDBUT_PIN) == LOW) {
      minuteconfirm++;
    }
    // updatedisplay
    showtime(hour, minute);

    delay(200);
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

  //Set theta1, theta2, phi1, phi2 (close,far,left,right)

  //LiDAR
  //scan the area where the laser is allowed to point

  //Turn LiDAR on

  //Horizontal motor will be moving from phi1(Leftmost point) to phi2(Rightmost point)
  //Vertical motor will be moving from theta1(closest point) to theta2(farthest point)

  //compare LiDAR data with calculated distance for each point


  //kinematics function here generate array
  int length = 50;
  int array[100][50];
  memset(array, 0, sizeof(array));
  f.kinematics(length, array, p1_phi, p1_theta, p2_phi, p2_theta, heightOfPost);
  //write the final array to eeprom
}



//Upon pressing confirm move to
//Inactive State




//After calibration is done
//Upon pressing confirm or reaching the set time move to
//Active State

//Turn the cat laser on
//Pathing Algorithm here
//Move motors

//While in Idle state if pressure is detected move to active state




void loop() {
  /*
  // put your main code here, to run repeatedly:
  digitalWrite(LASERCONTROL_PIN, HIGH);
  if (MOTION_IO_PIN == HIGH) {
    //Turn Laser On


    int seed = 0;
    while (MOTION_IO_PIN == HIGH) {
      randomSeed(seed);


      int s_x = random(0, length);
      int s_y = random(0, length * 2);

      // Ensure the starting point is on a valid path (1)
      while (array[s_y][s_x] != 1) {
        s_x = random(0, length);
        s_y = random(0, length * 2);
      }
      int phi = f.find_Phi(s_x / 10, s_y / 10);
      int theta = f.find_Theta(s_x / 10, s_y / 10, heightOfPost);
      setMotor(phi, theta);

      // Loop to generate the path
      
      int i = 0;
      while (i < 10 && MOTION_IO_PIN == HIGH) {
        i++;

        int adj_array[8][2] = { { s_y - 1, s_x - 1 }, { s_y - 1, s_x }, { s_y - 1, s_x + 1 }, { s_y, s_x - 1 }, { s_y, s_x + 1 }, { s_y + 1, s_x - 1 }, { s_y + 1, s_x }, { s_y + 1, s_x + 1 } };

        // Pick a random point from the adjacent array and set it as the current point
        // then find the adjacent array for that point.
        // Repeat these steps until a valid path (1) is found.
        int adj_point = random(0, 8);
        s_x = adj_array[adj_point][1];
        s_y = adj_array[adj_point][0];

        while (array[s_y][s_x] != 1) {
          adj_point = random(0, 8);
          s_x = adj_array[adj_point][1];
          s_y = adj_array[adj_point][0];
        }
        int phi = f.find_Phi(s_x / 10, s_y / 10);
        int theta = f.find_Theta(s_x / 10, s_y / 10, heightOfPost);
        setMotor(phi, theta)
        
        seed++;
      
    }
  }*/
}