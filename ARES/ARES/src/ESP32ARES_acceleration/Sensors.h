#pragma once

#include <Wire.h>
#include <ICM45686.h> // TDK Balanced-Gyro Driver

#include "Vector3.h"

class IMUs
{
	static int BootBMI();
	static int BootTDK();

	static void GetReadingsBMI(Vector3& outAccel, Vector3& outGyro);
	static void GetReadingsTDK(Vector3& outAccel, Vector3& outGyro);

private:
	static void writeReg(uint8_t dev, uint8_t reg, uint8_t val);

private:
	// Instantiate official TDK constructor interface targeting standard I2C bus
	ICM456xx m_ICM{ Wire, 1 };
};
