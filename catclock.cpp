#include "catclock.h"
#include "Arduino.h"
#include <TimeLib.h>
#include "LedControl.h"

cclock::cclock() {}

//byte dig[] = { 0b10111111, 0b10000110, 0b11011011, 0b11001111, 0b11100110, 0b11101101, 0b11111101, 0b10000111, 0b11111111, 0b11100111 };

void cclock::showdigit(int pos, int digit, byte array[]) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(dig[digit], i)) lc.setLed(0, i, pos, true);
  }
}

void cclock::showtime(int hours, int mins, byte array[]) {
  lc.clearDisplay(0);

  int first = hours / 10;
  int second = hours % 10;
  int third = mins / 10;
  int fourth = mins % 10;
  showdigit(1, first, array[]);
  showdigit(2, second, array[]);
  showdigit(3, third, array[]);
  showdigit(4, fourth, array[]);
}
