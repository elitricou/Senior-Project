#include "LedControl.h"
LedControl lc=LedControl(11,13,10,1);

byte dig[]={0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,0b01101101,0b01111101,0b00000111,0b01111111,0b01100111};
byte alp[]={0b01110111,0b01111111,0b00111001,0b01011110,0b01111001,0b01110001,0b00111101,0b01110110,0b00110000,0b00011111,0b01110101,0b00111000,0b00010101,0b00110111,0b00111111,0b01110011,0b01100111,0b00110011,0b01101101,0b01111000,0b00111110,0b00101110,0b00101010,0b01110110,0b01101110,0b01001011};

void setup() {
lc.shutdown(0,false); lc.setIntensity(0,8); lc.clearDisplay(0);
}

void showdigit(int pos,int digit){ for (int i=0; i < 7; i++){if (bitRead(dig[digit],i)) lc.setLed(0,i,pos,true);}}
void showletter(int pos,int digit){ for (int i=0; i < 7; i++){if (bitRead(alp[digit],i)) lc.setLed(0,i,pos,true);}}
void shownumber(int num){ lc.clearDisplay(0); String snum=String(num); for(int i=0;i<snum.length();i++){ int d = (int)snum.charAt(i)-48; showdigit(i+5-snum.length(),d);}}
void showword(String s){ lc.clearDisplay(0); String abc="abcdefghijklmnopqrstuvwxyz";for(int i=0;i<s.length();i++){showletter(i+5-s.length(),abc.indexOf(s.charAt(i)));}}

void loop() { 

// some tests

for(int l=-5;l<21;l++){lc.clearDisplay(0);for(int i=l;i<l+6;i++){ showletter(i-l-1,i);}delay(200);}

showword("one");
delay(500);
showword("two");
delay(500);
showword("thre");
delay(500);
showword("four");
delay(500);

for(int i=0;i<9999;i++){shownumber(i);delay(50);}

}