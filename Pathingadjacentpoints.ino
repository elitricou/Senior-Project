#include "functions.h"
#include <math.h>

// Function to generate a random number between min and max (inclusive)

functions f;
int length = 48;
int i = 0;
int j = 0;
int rows=96;
int cols=48;

float heightOfPost = 0.75;
// Function to create a path between adjacent points


void setup() {
  Serial.begin(9600);

  float p1_phi = 10;
  float p1_theta = 9;
  float p2_phi = 160;
  float p2_theta = 50;
  int array[48];

  for (int i = 0; i < length * 2; i++) {
    memset(array, 0, sizeof(array));
    f.kinematics(length, array, p1_phi, p1_theta, p2_phi, p2_theta, heightOfPost, i);
    f.writeArrEeprom(length, array, i);
  }

  
  randomSeed(analogRead(0));  // Seed the random number generator with an analog input value


  //set the initial point inside set up;
  i = random(0, 96);
  j = random(0, 48);
  f.readArrEeprom(length, array, i);
  while (array[j] != 1) {
    i = random(0, 96);
    j = random(0, 48);
    f.readArrEeprom(length, array, i);
  }
  Serial.println(i);
  Serial.println(j);
}

void loop() {
  // Call the function to create the path repeatedly
  int read_array[48];
  memset(read_array, 0, sizeof(read_array));
  int availablePaths = 0;

  f.readArrEeprom(length, read_array, i - 1);
  if (i > 0 && read_array[j] == 1) {
    availablePaths++;
  }
  f.readArrEeprom(length, read_array, i + 1);
  if (i < rows - 1 && read_array[j] == 1) {
    availablePaths++;
  }
  f.readArrEeprom(length, read_array, i);
  if (j > 0 && read_array[j - 1] == 1) {
    availablePaths++;
  }
  if (j < cols - 1 && read_array[j + 1] == 1) {
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
    if (i < rows - 1 && read_array[j] == 1 && randomDirection == 1) {
      i = i + 1;
    }
    f.readArrEeprom(length, read_array, i);
    if (j > 0 && read_array[j - 1] == 1 && randomDirection == 2) {
      j = j - 1;
    }
    if (j < cols - 1 && read_array[j + 1] == 1 && randomDirection == 3) {
      j = j + 1;
    }
  }
  f.readArrEeprom(length, read_array, i);
  Serial.print(i);
  Serial.print(", ");
  Serial.println(j);
  Serial.println(read_array[j]);


  delay(2000);  // Add a delay to make it more readable and to avoid flooding the serial monitor
}