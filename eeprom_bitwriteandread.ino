
#include <EEPROM.h>
void writeArrEeprom(int length, int array[]) {
  byte EEPROMbyte;
  for (int j = 0; j < length / 8; j++) {
    EEPROMbyte = EEPROM.read(j);
    for (int i = 0; i < 8; i++) {
      bitWrite(EEPROMbyte, i, array[i + 8 * j]);
    }
    EEPROM.update(j, EEPROMbyte);
  }
}

void readArrEeprom(int length, int array[]) {
  boolean pinState;
  for (int j = 0; j < length/8; j++) {
    for (int i = 0; i < 8; i++) {
      pinState = bitRead(EEPROM.read(j), i);  //where indices are j (x_coordinate)
      array[i + 8 * j] = pinState;
    }
  }
}

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/

void setup() { /** Empty setup. **/
  Serial.begin(9600);

  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }

  int array[48] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  
  int length=48;
  writeArrEeprom(length, array);

  //read
  boolean pinState;
  int read_array[48];
  memset(read_array, 0, sizeof(read_array));

  readArrEeprom(length, read_array);
  for (int x = 0; x < 48; x++) {
    Serial.print(read_array[x]);
    Serial.print(" ");
  }
  Serial.println();

}

void loop() {
}
