#define WIDTH 100
#define HEIGHT 50

#define XMAX ((WIDTH+7)/8)
#define YMAX HEIGHT

uint8_t data[XMAX][YMAX];

bool get_point(uint8_t x, uint8_t y){
	uint8_t a = x / 8;
	uint8_t b = x % 8;
	uint8_t d = data[a][y];
	return d & (1 << b);
}

void set_point(uint8_t x, uint8_t y, bool state){
	uint8_t a = x / 8;
	uint8_t b = x % 8;
	data[a][y] |= (1 << b);
}

void setup(){
	Serial.begin(9600);
	volatile int whatever;
	unsigned long start = millis();
	for(uint8_t i=0; i<WIDTH; i++){
		for(uint8_t j=0; j<HEIGHT; j++){
			whatever = get_point(i, j);
			set_point(i, j, !whatever);
		}
	}
	unsigned long end = millis();
	Serial.print("Bytes taken: ");
	Serial.print(sizeof(data));
	Serial.println();
	Serial.print("Time taken: ");
	Serial.print(end - start);
	Serial.println("ms");
}

void loop(){
}
