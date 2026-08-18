#pragma once

#include "Vector3.h"
#include "Quaternion.h"

#include "KalmanFilter.h"
#include "PID.h"

#include <stdint.h>

#include <LittleFS.h>

extern void OUTPUT_TEXT_ARES(const char* txt);
extern void OUTPUT_FLOAT_ARES(float num, uint8_t dp);

struct __attribute__((packed)) SaveData
{
	float TimeStamp;
	KalmanFilterState CurrentState;
	KalmanFilterSensorReadings SensorReadings;
	KalmanFilterProcessInputs ProcessInputs;
	PID_State PIDState;
};

struct SaveDataBuffer
{
	// constexpr uint8_t Length = 10;
	static const uint8_t LENGTH = 1;  // Temporary test! :)
	SaveData Data[LENGTH];

	uint8_t WriteIndex;
};

class FileSerialiser
{
public:
	FileSerialiser(const char* filename)
		: m_Filename(filename) {}

public:
	bool Init();
	void Close() { m_File.close(); }

	// Submit data to be appended to logs
	void Submit(const SaveDataBuffer& m_SaveDataBuffer);

	// Output all content to serial 
	void OutAll();

private:
	const char* m_Filename;
	fs::File m_File;
};
