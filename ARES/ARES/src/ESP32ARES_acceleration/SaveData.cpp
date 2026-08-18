#include "SaveData.h"

bool FileSerialiser::Init()
{
  if (!LittleFS.begin(true))
  {
    OUTPUT_TEXT_ARES("LittleFS Mount Failed.");
    return false;
  }

	m_File = LittleFS.open(m_Filename, FILE_APPEND);
  if (!m_File)
  {
    OUTPUT_TEXT_ARES("Failed to open file for writing [mode=appending].");
    return false;
  }

  return true;
}

void FileSerialiser::Submit(const SaveDataBuffer& dataBuffer)
{
  size_t bytesWritten = m_File.write((const uint8_t*)dataBuffer.Data, SaveDataBuffer::LENGTH * sizeof(SaveData));
  if (bytesWritten != SaveDataBuffer::LENGTH * sizeof(SaveData))
    OUTPUT_TEXT_ARES("File Write Error: Incomplete Data Written.");
  else
    OUTPUT_TEXT_ARES("SUCCESS: Data logged!");

  m_File.flush();
}

void FileSerialiser::OutAll()
{
  fs::File file = LittleFS.open(m_Filename);
  if (!file)
  {
    OUTPUT_TEXT_ARES("Failed to open file for reading.");
    return;
  }

  SaveData data;
  while (file.available() >= sizeof(SaveData))
  {
    size_t bytesRead = file.read((uint8_t*)&data, sizeof(SaveData));
    
    #define OUTPUT_CSV_VEC(v) v.PrintRaw(); OUTPUT_TEXT_ARES(", ")
    #define OUTPUT_CSV_FLOAT(v) OUTPUT_FLOAT_ARES(v, 3); OUTPUT_TEXT_ARES(", ")

    OUTPUT_FLOAT_ARES(data.TimeStamp, 3);
    OUTPUT_TEXT_ARES(", ");
    // Current State
    OUTPUT_CSV_VEC(data.CurrentState.Position);
    OUTPUT_CSV_VEC(data.CurrentState.Velocity);
    OUTPUT_CSV_VEC(data.CurrentState.Orientation);
    OUTPUT_CSV_VEC(data.CurrentState.BiasMeanAccel);
    OUTPUT_CSV_VEC(data.CurrentState.BiasMeanGyro);
    OUTPUT_CSV_VEC(data.CurrentState.BiasDeltaAccel);
    OUTPUT_CSV_VEC(data.CurrentState.BiasDeltaGyro);
    // IMUs
    OUTPUT_CSV_VEC(data.ProcessInputs.Accel1);
    OUTPUT_CSV_VEC(data.ProcessInputs.Gyro1);
    OUTPUT_CSV_VEC(data.ProcessInputs.Accel2);
    OUTPUT_CSV_VEC(data.ProcessInputs.Gyro2);
    // Sensors
    OUTPUT_CSV_VEC(data.SensorReadings.GPS);
    OUTPUT_CSV_VEC(data.SensorReadings.Mag);
    OUTPUT_CSV_VEC(data.SensorReadings.DeltaAccel);
    OUTPUT_CSV_VEC(data.SensorReadings.DeltaGyro);
#if BAROMETER_ENABLED
    OUTPUT_CSV_FLOAT(data.SensorReadings.Barom);
#endif
    // PID
    OUTPUT_CSV_VEC(data.PIDState.Target);
    OUTPUT_CSV_VEC(data.PIDState.TargetOffset);
    OUTPUT_CSV_VEC(data.PIDState.TargetHeading);
    OUTPUT_CSV_VEC(data.PIDState.ServoOrientation);
    OUTPUT_CSV_FLOAT(data.PIDState.Thrust);
    
    #undef OUTPUT_CSV_FLOAT
    #undef OUTPUT_CSV_VEC

    if (bytesRead != sizeof(SaveData))
      OUTPUT_TEXT_ARES("File Write Error: Incomplete Data Written.");
  }
}
