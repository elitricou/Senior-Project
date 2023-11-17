#ifndef catclock_h
#define catclock_h

class cclock {
public:
	cclock();
	
	void showdigit(int pos, int digit, byte array[]);
	void showtime(int hours, int mins, byte array[])
};

#endif