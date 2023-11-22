// Using an example file modified by Madrajib for a 4 coomon anode 7 segment display.....
#include <TimeLib.h>
#include "LedControl.h"
// Clock LED by Elisabeth Tricou


//Clock pins
#define CLK_PIN PIN_PC5
#define CLOCK_CS PIN_PD2
#define DISPLAY_PIN PIN_PD4
//buttons
#define UPBUT_PIN PIN_PC0
#define DOWNBUT_PIN PIN_PC1
#define MIDBUT_PIN PIN_PC4
//Laser
#define LASERCONTROL_PIN PIN_PB0
//Pressure sensor
#define PRESSURE_PIN PIN_PB2
//Motion sensor
#define MOTION_IO_PIN PIN_PB7 

LedControl lc = LedControl(DISPLAY_PIN, CLK_PIN, CLOCK_CS, 1);
byte dig[] = { 0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111 };
byte alp[] = { 0b01110111, 0b01111111, 0b00111001, 0b01011110, 0b01111001, 0b01110001, 0b00111101, 0b01110110, 0b00110000, 0b00011111, 0b01110101, 0b00111000, 0b00010101, 0b00110111, 0b00111111, 0b01110011, 0b01100111, 0b00110011, 0b01101101, 0b01111000, 0b00111110, 0b00101110, 0b00101010, 0b01110110, 0b01101110, 0b01001011 };

void showletter(int pos, int digit) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(alp[digit], i)) lc.setLed(0, i, pos, true);
  }
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
  showdigit(2, second, false);
  showdigit(3, third, true);
  showdigit(4, fourth, true);
}
int h = 0;
int m = 0;
int s_h = 0;
int s_m = 0;
int e_h = 0;
int e_m = 0;
void setup() {

  //Test whether or not pins have to be set as outputs
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CLOCK_CS, OUTPUT);
  pinMode(DISPLAY_PIN, OUTPUT);

  pinMode(UPBUT_PIN, INPUT);
  pinMode(DOWNBUT_PIN, INPUT);
  pinMode(MIDBUT_PIN, INPUT);

  pinMode(LASERCONTROL_PIN,OUTPUT);
  digitalWrite(LASERCONTROL_PIN, HIGH);

  pinMode(PRESSURE_PIN,INPUT);

  pinMode(MOTION_IO_PIN,INPUT);

  digitalWrite(UPBUT_PIN, HIGH);
  digitalWrite(DOWNBUT_PIN, HIGH);
  digitalWrite(MIDBUT_PIN, HIGH);


  lc.shutdown(0, false);  //to take the LED's out of power-down mode
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);  //Switch all Leds on the display off

  set_time();

  showtime(0, 0);
  delay(200);

  lc.clearDisplay(0);
  showletter(1, 17);  //R
  showletter(2, 4);   //E
  showletter(3, 0);   //A
  showletter(4, 11);  //L
  delay(2000);


  int hour = 0;
  int minute = 0;
  while (digitalRead(MIDBUT_PIN) != LOW) {
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
    showtime(hour, 0);
    delay(200);
  }

  showtime(0, 0);
  delay(200);


  while (digitalRead(MIDBUT_PIN) != LOW) {
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
    showtime(0, minute);
    delay(200);
  }
  h = hour;
  m = minute;
  
  //START
  showtime(0, 0);
  delay(200);

  lc.clearDisplay(0);
  showletter(1, 18);  //S
  showletter(2, 19);   //T
  showletter(3, 17);   //R
  showletter(4, 19);  //T
  delay(2000);

  int strhour = 0;
  int strminute = 0;
  while (digitalRead(MIDBUT_PIN) != LOW) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      strhour++;
      if (strhour == 24) {
        strhour = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      strhour--;
      if (strhour == -1) {
        strhour = 23;
      }
    }
    showtime(strhour, 0);
    delay(200);
  }

  showtime(0, 0);
  delay(200);


  while (digitalRead(MIDBUT_PIN) != LOW) {
    if (digitalRead(UPBUT_PIN) == LOW) {
      strminute++;
      if (strminute == 60) {
        strminute = 0;
      }
    }
    if (digitalRead(DOWNBUT_PIN) == LOW) {
      strminute--;
      if (strminute == -1) {
        strminute = 59;
      }
    }
    showtime(0, strminute);
    delay(200);
  }

  showtime(strhour, strminute);
  delay(2000);

  s_h=strhour;
  s_m=strminute;

  //END
  showtime(0, 0);
  delay(200);

  lc.clearDisplay(0);
  showletter(1, 4);  //E
  showletter(2, 13);   //N
  showletter(3, 3);   //D
  delay(2000);

  int endhour = 0;
  int endminute = 0;
  while (digitalRead(MIDBUT_PIN) != LOW) {
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
    showtime(endhour, 0);
    delay(200);
  }

  showtime(0, 0);
  delay(200);


  while (digitalRead(MIDBUT_PIN) != LOW) {
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
    showtime(0, endminute);
    delay(200);
  }

  showtime(endhour, endminute);
  delay(2000);
  e_h=endhour;
  e_m=endminute;



}
int curr_time_h=0;
int curr_time_m=0;
int prev_time_m=0;
int diff_m=0;

void loop() {
  curr_time_h=h+hour();
  curr_time_m=m+minute();
  prev_time_m=curr_time_m;
  diff_m=prev_time_m-curr_time_m;
  showtime(curr_time_h, curr_time_m);


  //at start time the laser turns on
  // if(curr_time_h==s_h&&curr_time_m==s_m){
  //   digitalWrite(LASERCONTROL_PIN, LOW);
  // }

  //Pressure sensor test
  // if(digitalRead(PRESSURE_PIN)==HIGH){
  //   digitalWrite(LASERCONTROL_PIN, LOW);
  // }

  if(digitalRead(MOTION_IO_PIN_PIN)==HIGH){
    showtime(1, 1);
    delay(2000);
  }


}


//What does it do?
void set_time() {
  byte minutes1 = 0;
  byte hours1 = 0;
  byte minutes = minute();
  byte hours = hour();
  minutes1 = minutes;
  hours1 = hours;
}