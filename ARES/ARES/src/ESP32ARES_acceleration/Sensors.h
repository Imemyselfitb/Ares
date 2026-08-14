#pragma once

#include <Wire.h>
#include <ICM45686.h> // TDK Balanced-Gyro Driver

#include "Vector3.h"

namespace IMUs
{
	int BootBMI();
	int BootTDK();

	void GetReadingsBMI(Vector3& outAccel, Vector3& outGyro);
	void GetReadingsTDK(Vector3& outAccel, Vector3& outGyro);
};
