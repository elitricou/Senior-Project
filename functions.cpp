#include "functions.h"
#include <math.h>
#include <Arduino.h> 

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
float functions::calcDistance(float row, float col, float heightOfPost){
  row=row/10;
  col=col/10;
  float distfrombase=sqrt(pow(row,2)+pow(col,2));
  return sqrt(pow(heightOfPost,2)+pow(distfrombase,2));
}
functions::Coordinate functions::calculateCoordinates(int phi, int theta, float heightOfPost) {
    Coordinate result;
    result.x = find_X(phi, theta, heightOfPost);
    result.y = find_Y(phi, theta, heightOfPost);
    return result;
}
void functions::kinematics(int length, int array[][50], int p1_phi, int p1_theta, int p2_phi, int p2_theta, float heightOfPost) {
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

    for (int x = 0; x < length; x++) {
        for (int y = -length; y < length; y++) {
            float circle_eq = sqrt(pow(x, 2) + pow(y, 2));
            float right_line_eq = x * right_line_slope;
            float left_line_eq = x * left_line_slope;

            if (circle_eq < outer_radius * 10.0 && circle_eq > inner_radius * 10.0 && y < right_line_eq && y > left_line_eq) {
                array[y + length][x] = 1;
            }
        }
    }
}


}