#include "BMX055.h"

#define FIFO_CONFIG_0 0x30
#define FIFO_CONFIG_1 0x3e
#define FIFO_DATA 0x3f
#define INT_EN_1 0x17
#define INT_STATUS_1 0x0a

BMX055::BMX055(){}

uint8_t BMX055::beginAcc(char range)
{

	char _range = range; //2g Range 0x03

	switch (range)
	{
	case 0x03:
		accRange = (2.0 / 2048.0);
		break;
	case 0x05:
		accRange = (4.0 / 2048.0);
		break;
	case 0x08:
		accRange = (8.0 / 2048.0);
		break;
	case 0x0C:
		accRange = (16.0 / 2048.0);
		break;
	}
	// Initialise I2C communication as MASTER
	Wire.begin();
	Wire.setClock(400000);

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_ACCL_ADDR);
	// Select PMU_Range register
	Wire.write(0x0F);
	// Range = +/- 2g
	Wire.write(range);
	// Stop I2C Transmission
	Wire.endTransmission();

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_ACCL_ADDR);
	// Select PMU_BW register
	Wire.write(0x10);
	// Bandwidth = 1 kHz
	Wire.write(0x08);
	// Stop I2C Transmission
	Wire.endTransmission();
	
	Wire.beginTransmission(BMX055_ACCL_ADDR);
	Wire.write(FIFO_CONFIG_0);
	// Set FIFO watermark level to 5 frames
	Wire.write(0x05);
	Wire.endTransmission();
	
	Wire.beginTransmission(BMX055_ACCL_ADDR);
	Wire.write(INT_EN_1);
	// Enable FIFO watermark interrupt
	Wire.write(0x40);
	Wire.endTransmission();
	
	Wire.beginTransmission(BMX055_ACCL_ADDR);
	Wire.write(FIFO_CONFIG_1);
	// STREAM fifo with X, Y and Z
	Wire.write(0x80);
	Wire.endTransmission();
}

uint8_t BMX055::beginGyro()
{

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_GYRO_ADDR);
	// Select Range register
	Wire.write(0x0F);
	// Full scale = +/- 125 degree/s
	Wire.write(0x04);
	// Stop I2C Transmission
	Wire.endTransmission();

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_GYRO_ADDR);
	// Select Bandwidth register
	Wire.write(0x10);
	// ODR = 100 Hz
	Wire.write(0x07);
	// Stop I2C Transmission
	Wire.endTransmission();

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_GYRO_ADDR);
	// Select LPM1 register
	Wire.write(0x11);
	// Normal mode, Sleep duration = 2ms
	Wire.write(0x00);
	// Stop I2C Transmission
	Wire.endTransmission();
}

uint8_t BMX055::beginMagn()
{

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_MAGN_ADDR);
	// Select Mag register
	Wire.write(0x4B);
	// Soft reset
	Wire.write(0x83);
	// Stop I2C Transmission
	Wire.endTransmission();

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_MAGN_ADDR);
	// Select Mag register
	Wire.write(0x4C);
	// Normal Mode, ODR = 10 Hz
	Wire.write(0x00);
	// Stop I2C Transmission
	Wire.endTransmission();

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_MAGN_ADDR);
	// Select Mag register
	Wire.write(0x4E);
	// X, Y, Z-Axis enabled
	Wire.write(0x84);
	// Stop I2C Transmission
	Wire.endTransmission();

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_MAGN_ADDR);
	// Select Mag register
	Wire.write(0x51);
	// No. of Repetitions for X-Y Axis = 9
	Wire.write(0x04);
	// Stop I2C Transmission
	Wire.endTransmission();

	// Start I2C Transmission
	Wire.beginTransmission(BMX055_MAGN_ADDR);
	// Select Mag register
	Wire.write(0x52);
	// No. of Repetitions for Z-Axis = 15
	Wire.write(0x0F);
	// Stop I2C Transmission
	Wire.endTransmission();
	delay(300);
}

void BMX055::getAcceleration(float *x, float *y, float *z, float *accTotal)
{

	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_ACCL_ADDR);
		// Select data register
		Wire.write((2 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_ACCL_ADDR, 1);
		// Read 6 bytes of data
		// xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	// Convert the data to 12-bits
	int xAccl = ((_data[1] * 256) + (_data[0] & 0xF0)) / 16;
	if (xAccl > 2047)
		xAccl -= 4096;
	*x = xAccl * accRange * 9.81f;

	int yAccl = ((_data[3] * 256) + (_data[2] & 0xF0)) / 16;
	if (yAccl > 2047)
		yAccl -= 4096;
	*y = yAccl * accRange * 9.81f;

	int zAccl = ((_data[5] * 256) + (_data[4] & 0xF0)) / 16;
	if (zAccl > 2047)
		zAccl -= 4096;
	*z = zAccl * accRange * 9.81f;

	*accTotal = sqrt((sq(*x) + sq(*y) + sq(*z)));
}

bool BMX055::canRead5Frames() {
	Wire.beginTransmission(BMX055_ACCL_ADDR);
	Wire.write(INT_STATUS_1);
	Wire.endTransmission();
	Wire.requestFrom(BMX055_ACCL_ADDR, 1);
	return Wire.read() & 0x40;
}

void BMX055::readFIFO5(short* xyz5) {
	Wire.beginTransmission(BMX055_ACCL_ADDR);
	Wire.write(FIFO_DATA);
	Wire.endTransmission();
	Wire.requestFrom(BMX055_ACCL_ADDR, 30);	// 5 frames
	for(byte f = 0; f < 5; ++f) {
		for(byte i = 0; i < 6; ++i) {
			_data[i] = Wire.read();
		}
		// Convert the data to 12-bits
		xyz5[3 * f] = ((_data[1] * 256) + (_data[0] & 0xF0)) / 16;
		if (xyz5[3 * f] > 2047)
			xyz5[3 * f] -= 4096;

		xyz5[3 * f + 1] = ((_data[3] * 256) + (_data[2] & 0xF0)) / 16;
		if (xyz5[3 * f + 1] > 2047)
			xyz5[3 * f + 1] -= 4096;

		xyz5[3 * f + 2] = ((_data[5] * 256) + (_data[4] & 0xF0)) / 16;
		if (xyz5[3 * f + 2] > 2047)
			xyz5[3 * f + 2] -= 4096;
	}
}

void BMX055::getAcceleration(short *x, short *y, short *z)
{

	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_ACCL_ADDR);
		// Select data register
		Wire.write((2 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_ACCL_ADDR, 1);
		// Read 6 bytes of data
		// xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	// Convert the data to 12-bits
	*x = ((_data[1] * 256) + (_data[0] & 0xF0)) / 16;
	if (*x > 2047)
		*x -= 4096;

	*y = ((_data[3] * 256) + (_data[2] & 0xF0)) / 16;
	if (*y > 2047)
		*y -= 4096;

	*z = ((_data[5] * 256) + (_data[4] & 0xF0)) / 16;
	if (*z > 2047)
		*z -= 4096;
}

float BMX055::getAccelerationX()
{
	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_ACCL_ADDR);
		// Select data register
		Wire.write((2 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_ACCL_ADDR, 1);
		// Read 6 bytes of data
		// xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	// Convert the data to 12-bits
	int xAccl = ((_data[1] * 256) + (_data[0] & 0xF0)) / 16;
	if (xAccl > 2047)
		xAccl -= 4096;
	float x = xAccl * accRange;
	return x;
}

float BMX055::getAccelerationY()
{
	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_ACCL_ADDR);
		// Select data register
		Wire.write((2 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_ACCL_ADDR, 1);
		// Read 6 bytes of data
		// xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	int yAccl = ((_data[3] * 256) + (_data[2] & 0xF0)) / 16;
	if (yAccl > 2047)
		yAccl -= 4096;
	float y = yAccl * accRange;
	return y;
}

float BMX055::getAccelerationZ()
{
	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_ACCL_ADDR);
		// Select data register
		Wire.write((2 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_ACCL_ADDR, 1);
		// Read 6 bytes of data
		// xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	int zAccl = ((_data[5] * 256) + (_data[4] & 0xF0)) / 16;
	if (zAccl > 2047)
		zAccl -= 4096;
	float z = zAccl * accRange;
	return z;
}

float BMX055::getAccelerationTotal()
{

	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_ACCL_ADDR);
		// Select data register
		Wire.write((2 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_ACCL_ADDR, 1);
		// Read 6 bytes of data
		// xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	// Convert the data to 12-bits
	int xAccl = ((_data[1] * 256) + (_data[0] & 0xF0)) / 16;
	if (xAccl > 2047)
		xAccl -= 4096;
	float x = xAccl * accRange * 9.81f;

	int yAccl = ((_data[3] * 256) + (_data[2] & 0xF0)) / 16;
	if (yAccl > 2047)
		yAccl -= 4096;
	float y = yAccl * accRange * 9.81f;

	int zAccl = ((_data[5] * 256) + (_data[4] & 0xF0)) / 16;
	if (zAccl > 2047)
		zAccl -= 4096;
	float z = zAccl * accRange * 9.81f;

	float accTotal = sqrt((sq(x) + sq(y) + sq(z)));
	return accTotal;
}

void BMX055::getMagnet(int *x, int *y, int *z)
{

	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_MAGN_ADDR);
		// Select data register
		Wire.write((66 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_MAGN_ADDR, 1);
		// Read 6 bytes of data
		// xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	// Convert the data
	int xMag = ((_data[1] * 256) + (_data[0] & 0xF8)) / 8;
	if (xMag > 4095)
		xMag -= 8192;
	*x = xMag;

	int yMag = ((_data[3] * 256) + (_data[2] & 0xF8)) / 8;
	if (yMag > 4095)
		yMag -= 8192;
	*y = yMag;

	int zMag = ((_data[5] * 256) + (_data[4] & 0xFE)) / 2;
	if (zMag > 16383)
		zMag -= 32768;
	*z = zMag;
}
void BMX055::getRotation(int *x, int *y, int *z)
{

	for (int i = 0; i < 6; i++)
	{
		// Start I2C Transmission
		Wire.beginTransmission(BMX055_GYRO_ADDR);
		// Select data register
		Wire.write((2 + i));
		// Stop I2C Transmission
		Wire.endTransmission();
		// Request 1 byte of data
		Wire.requestFrom(BMX055_GYRO_ADDR, 1);
		// Read 6 bytes of data
		// xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
		if (Wire.available() == 1)
			_data[i] = Wire.read();
	}

	// Convert the data
	int xGyro = (_data[1] * 256) + _data[0];
	if (xGyro > 32767)
		xGyro -= 65536;
	*x = xGyro;

	int yGyro = (_data[3] * 256) + _data[2];
	if (yGyro > 32767)
		yGyro -= 65536;
	*y = yGyro;

	int zGyro = (_data[5] * 256) + _data[4];
	if (zGyro > 32767)
		zGyro -= 65536;
	*z = zGyro;
}