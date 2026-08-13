#pragma once

#include <Wire.h>
#include <ICM45686.h> // TDK Balanced-Gyro Driver

#include "Vector3.h"

struct IMUs
{
	int BootBMI();
	int BootTDK();

	void GetReadingsBMI(Vector3& outAccel, Vector3& outGyro);
	void GetReadingsTDK(Vector3& outAccel, Vector3& outGyro);

private:
	void writeReg(uint8_t dev, uint8_t reg, uint8_t val);

private:
	// Instantiate official TDK constructor interface targeting standard I2C bus
	ICM456xx m_ICM{ Wire, 1 };
};
