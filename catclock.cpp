#include "catclock.h"
#include "Arduino.h"
#include <TimeLib.h>
#include "LedControl.h"

cclock::cclock() {}

void cclock::showdigit(int pos, int digit) {
  for (int i = 0; i < 7; i++) {
    if (bitRead(dig[digit], i)) lc.setLed(0, i, pos, true);
  }
}

void cclock::showtime(int hours, int mins) {
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
