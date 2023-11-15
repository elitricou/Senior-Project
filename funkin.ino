
#include <math.h>

const float heightOfPost = 0.75;

struct Coordinate {
  float x;
  float y;
};

float findX(int phi, int theta) {
  float distanceFromBase = heightOfPost / tan(radians(theta));
  return distanceFromBase * sin(radians(phi));
}

float findY(int phi, int theta) {
  float distanceFromBase = heightOfPost / tan(radians(theta));
  return distanceFromBase * cos(radians(phi));
}

Coordinate calculateCoordinates(int phi, int theta) {
  Coordinate result;
  result.x = findX(phi, theta);
  result.y = findY(phi, theta);
  return result;
}

void kinematics(int length, int array[], int p1_phi, int p1_theta, int p2_phi, int p2_theta, int row) {
  Coordinate p1_coords = calculateCoordinates(p1_phi, p1_theta);
  Coordinate p2_coords = calculateCoordinates(p2_phi, p2_theta);

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

  for (int x = 0; x < length; x++) {
    float circle_radius = sqrt(pow(x, 2) + pow(row-length, 2));
    float right_line_eq = x * right_line_slope;
    float left_line_eq = x * left_line_slope;

    if (circle_radius < outer_radius * 10.0 && circle_radius > inner_radius * 10.0 && row-length < right_line_eq && row-length > left_line_eq) {
      array[x] = 1;
    }
  }
}

void setup() {
  Serial.begin(9600);

  int length = 48;
  float p1_phi = 10;
  float p1_theta = 9;
  float p2_phi = 160;
  float p2_theta = 50;
  int array[48];
  for (int i = 0; i < length * 2; i++) {
    memset(array, 0, sizeof(array));
    kinematics(length, array, p1_phi, p1_theta, p2_phi, p2_theta, i);
    for (int x = 0; x < length; x++) {
      Serial.print(array[x]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
void loop() {
  // put your main code here, to run repeatedly:
}
