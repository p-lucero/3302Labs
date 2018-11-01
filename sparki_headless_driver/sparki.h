#include <sys/time.h>
#include <cstddef>
#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#ifndef SPARKI_HEADLESS
#define SPARKI_HEADLESS 0

// defines for left and right motors
typedef unsigned char byte;
#define MOTOR_LEFT    0
#define MOTOR_RIGHT   1
#define MOTOR_GRIPPER 2

// defines for direction
#define DIR_CCW -1
#define DIR_CW   1

class Sparki {
public:
	void print(char* s){printf("%s", s);}
	void println(char* s){printf("%s\n", s);}

	void print(const char* s){printf("%s", s);}
	void println(const char* s){printf("%s\n", s);}

	void print(char i){printf("%c", i);}
	void println(char i){printf("%c\n", i);}

	void print(unsigned char i){printf("%hhu", i);}
	void println(unsigned char i){printf("%hhu\n", i);}

	void print(int i){printf("%d", i);}
	void println(int i){printf("%d\n", i);}

	void print(unsigned int i){printf("%u", i);}
	void println(unsigned int i){printf("%u\n", i);}

	void print(long i){printf("%ld", i);}
	void println(long i){printf("%ld\n", i);}

	void print(unsigned long i){printf("%lu", i);}
	void println(unsigned long i){printf("%lu\n", i);}

	void print(double i){printf("%f", i);}
	void println(double i){printf("%f\n", i);}

	float accelX(){return 0;}
	float accelY(){return 0;}
	float accelZ(){return 0;}

	void beep(){}
	void beep(int freq){}
	void beep(int freq, int time){}

	void gripperOpen(){}
	void gripperOpen(int width_cm){}
	void gripperClose(){}
	void gripperClose(int width_cm){}
	void gripperStop(){}

	void sendIR(){}

	int edgeLeft(){return 1000;}
	int lineLeft(){return 1000;}
	int lineCenter(){return 1000;}
	int lineRight(){return 1000;}
	int edgeRight(){return 1000;}

	void clearLCD(){}
	void updateLCD(){}
	 
	void drawPixel(int xPixel, int yPixel){}
	void readPixel(int xPixel, int yPixel){}
	void drawChar(int xPixel, int yLine, char c){}
	void drawString(int xPixel, int yLine, char* s){}
	void drawLine(int xStart, int yStart, int xEnd, int yEnd){}
	void drawRect(int xCenter, int yCenter, int width, int height){}
	void drawRectFilled(int xCenter, int yCenter, int width, int height){}
	void drawCircle(int xCenter, int yCenter, int radius){}
	void drawCircleFilled(int xCenter, int yCenter, int radius){}
	void drawBitmap(int xStart, int yStart, bool* bitmap, int width, int height){}

	void moveForward(int distance_centimeters){}
	void moveBackward(int distance_centimeters){}
	void moveLeft(int angle_degrees){}
	void moveRight(int angle_degrees){}
	void moveStop(){}

	void servo(int angle_degrees){}

	void motorRotate(int motor, int direction, int speed){}
	void motorStop(int motor){}

	int ping(){return 10;}
};

class serial {
public:
	void print(char* s){fprintf(stderr, "%s", s);}
	void println(char* s){fprintf(stderr, "%s\n", s);}

	void print(const char* s){fprintf(stderr, "%s", s);}
	void println(const char* s){fprintf(stderr, "%s\n", s);}

	void print(char i){fprintf(stderr, "%c", i);}
	void println(char i){fprintf(stderr, "%c\n", i);}

	void print(unsigned char i){fprintf(stderr, "%hhu", i);}
	void println(unsigned char i){fprintf(stderr, "%hhu\n", i);}

	void print(int i){fprintf(stderr, "%d", i);}
	void println(int i){fprintf(stderr, "%d\n", i);}

	void print(unsigned int i){fprintf(stderr, "%u", i);}
	void println(unsigned int i){fprintf(stderr, "%u\n", i);}

	void print(long i){fprintf(stderr, "%ld", i);}
	void println(long i){fprintf(stderr, "%ld\n", i);}

	void print(unsigned long i){fprintf(stderr, "%lu", i);}
	void println(unsigned long i){fprintf(stderr, "%lu\n", i);}

	void print(double i){fprintf(stderr, "%f", i);}
	void println(double i){fprintf(stderr, "%f\n", i);}
};

void delay(int ms){
	usleep(ms * 1000);
}

unsigned long millis(){
	// ripped from https://stackoverflow.com/a/10889489
	struct timeval tv;
	gettimeofday(&tv, NULL);

	unsigned long long millisecondsSinceEpoch =
    (unsigned long long)(tv.tv_sec) * 1000 +
    (unsigned long long)(tv.tv_usec) / 1000;
	return millisecondsSinceEpoch;
}

Sparki sparki;
serial Serial; // aaaaaaaaaaaaaa
#endif