#pragma once

#include <HardwareSerial.h>
#include "Quaternion.h"
#include "Vector3.h"
#include "Vector2.h"

#define I2C_SDA 4
#define I2C_SCL 5
inline bool g_IsWireInitialised = false;

namespace IMUs
{
	int BootBMI();
	int BootTDK();

	void GetReadingsBMI(Vector3& outAccel, Vector3& outGyro);
	void GetReadingsTDK(Vector3& outAccel, Vector3& outGyro);
}

namespace Magnetometer
{
	bool Init();
	bool GetReading(Vector3& outDirection);
}

enum class GPSFixQuality : uint8_t
{
	Invalid = 0, 
	GPS, 
	DGPS, 
	PPS, 
	RTK,
	FloatRTK,
	Estimated,
	Manual,
	Simulated
};

struct GPSData
{
	double Timestamp;
	double Latitude, Longitude, Altitude;
	GPSFixQuality Fix;
};

enum class LaunchState : uint8_t
{
	Idle = 0,
	Countdown,
	Ascending,
	Landing,
	Ground
};

struct __attribute__((__packed__)) TelemeteryData
{
	uint16_t PacketID;
	
	LaunchState CurrentLaunchState;
	uint32_t TimeStamp;

	Quaternion Orientation;
	Vector3 Position;
	float Velocity;
	float Accel;

	Vector2 ServoAngles;
	float Throttle;
};

// Includes setup of the tightly-linked GPS
class Telemetry
{
public:
 	void Init();
 	void ProcessIncoming();
	void ProcessGPS(GPSData& outData, Vector3& outGPSPosition);
	void EmitPacket(TelemeteryData& packet);

private:
	void handleIncomingCommand(uint8_t commandByte);

private:
 	HardwareSerial m_SerialTelem{ 1 };
 	HardwareSerial m_SerialGPS{ 2 };
};
