#ifndef Morse_h
#define Morse_h

// the #include statment and code go here...

#endif
#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

#define BMX055_ACCL_ADDR 0x19
#define BMX055_GYRO_ADDR 0x69
#define BMX055_MAGN_ADDR 0x13

class BMX055
{
public:
	BMX055();
	uint8_t beginAcc(char range);
	uint8_t beginGyro(void);
	uint8_t beginMagn(void);
	void getAcceleration(float *x, float *y, float *z, float *accTotal);
	void getAcceleration(short *x, short *y, short *z);
	float getAccelerationX();
	float getAccelerationY();
	float getAccelerationZ();
	float getAccelerationTotal();
	void getMagnet(int *x, int *y, int *z);
	void getRotation(int *x, int *y, int *z);
	
	bool canRead5Frames();
	void readFIFO5(short* xyz5);

private:
	unsigned int _data[6];
	char _range;
	float accRange;
};