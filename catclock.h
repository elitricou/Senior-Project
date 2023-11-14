#ifndef catclock_h
#define catclock_h

class cclock {
public:
	cclock();
	
	byte dig[] = { 0b10111111, 0b10000110, 0b11011011, 0b11001111, 0b11100110, 0b11101101, 0b11111101, 0b10000111, 0b11111111, 0b11100111 };
	
	void showdigit(int pos, int digit);
	void showtime(int hours, int mins);	
};

#endif