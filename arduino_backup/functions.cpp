#include "functions.h"
#include <math.h>
#include <Arduino.h>
#include <EEPROM.h>

functions::functions() {}

float functions::find_X(int phi, int theta, float heightOfPost) {
  float distanceFromBase = heightOfPost / tan(radians(theta));
  return distanceFromBase * sin(radians(phi));
}

float functions::find_Y(int phi, int theta, float heightOfPost) {
  float distanceFromBase = heightOfPost / tan(radians(theta));
  return distanceFromBase * cos(radians(phi));
}

float functions::find_Phi(float X, float Y) {
  return degrees(atan2(X, Y));
}

float functions::find_Theta(float X, float Y, float heightOfPost) {
  float distanceFromBase = sqrt(pow(X, 2) + pow(Y, 2));
  return degrees(atan2(heightOfPost, distanceFromBase));
}


double functions::calc_distance(int row, int col, float h) {
  float r = 1;             //radius of top motor arc
  float theta, y, rad, distance;  //total hieght, total base of triangle, hypotenuse

  row=row/10;
  col=col/10;

  float distfrombase=sqrt(pow(row,2)+pow(col,2));
  
  theta = atan2(distfrombase,h);

  y = r * sin(theta) + h;
  rad = h / cos(theta) + (r - r * cos(theta));

  distance = sqrt(pow(y,2) + pow(rad,2));

  return distance;
}




functions::Coordinate functions::calculateCoordinates(int phi, int theta, float heightOfPost) {
  Coordinate result;
  result.x = find_X(phi, theta, heightOfPost);
  result.y = find_Y(phi, theta, heightOfPost);
  return result;
}
void functions::kinematics(int length, int array[], int p1_phi, int p1_theta, int p2_phi, int p2_theta, float heightOfPost, int row) {
  Coordinate p1_coords = calculateCoordinates(p1_phi, p1_theta, heightOfPost);
  Coordinate p2_coords = calculateCoordinates(p2_phi, p2_theta, heightOfPost);

  float outer_radius, inner_radius;

  if (p1_theta < p2_theta) {
    outer_radius = sqrt(pow(p1_coords.x, 2) + pow(p1_coords.y, 2));
    inner_radius = sqrt(pow(p2_coords.x, 2) + pow(p2_coords.y, 2));
  } else {
    outer_radius = sqrt(pow(p2_coords.x, 2) + pow(p2_coords.y, 2));
    inner_radius = sqrt(pow(p1_coords.x, 2) + pow(p1_coords.y, 2));
  }

  float right_line_slope, left_line_slope;

  if (p1_phi < p2_phi) {
    right_line_slope = p1_coords.y / p1_coords.x;
    left_line_slope = p2_coords.y / p2_coords.x;
  } else {
    right_line_slope = p2_coords.y / p2_coords.x;
    left_line_slope = p1_coords.y / p1_coords.x;
  }

  for (int x = 0; x < length; x++){
    float circle_radius = sqrt(pow(x, 2) + pow(row-length, 2));
    float right_line_eq = x * right_line_slope;
    float left_line_eq = x * left_line_slope;

    if (circle_radius < outer_radius * 10.0 && circle_radius > inner_radius * 10.0 && row-length < right_line_eq && row-length > left_line_eq) {
      array[x] = 1;
    }
  }
}


void functions::writeArrEeprom(int length, int array[], int row) {
  byte EEPROMbyte;
  for (int j =0; j < length / 8; j++) {
    EEPROMbyte = EEPROM.read(j+row*length/8);
    for (int i = 0; i < 8; i++) {
      bitWrite(EEPROMbyte, i, array[i + 8*j]);
    }
    EEPROM.update(j+row*length/8, EEPROMbyte);
  }
}

void functions::readArrEeprom(int length, int array[], int row) {
  boolean pinState;
  for (int j = 0; j <length / 8; j++) {
    for (int i = 0; i < 8; i++) {
      pinState = bitRead(EEPROM.read(j+row*length/8), i);  //where indices are j (x_coordinate)
      array[i + 8*j] = pinState;
    }
  }
}

