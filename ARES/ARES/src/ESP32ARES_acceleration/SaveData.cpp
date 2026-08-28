#include "SaveData.h"

#include <Adafruit_TinyUSB.h>

// 512KB virtual USB flash
#define MSC_BLOCK_SIZE 512
#define MSC_DISK_BLOCK_COUNT 1024

struct MSC
{
  Adafruit_USBD_MSC usbMSC;
  bool IsActiveUSB;
  
  static int32_t ReadCallback(uint32_t logicalBlockAddress, void* buffer, uint32_t bufferSize);
};
static MSC s_MSC;
static const char* s_SaveLogsFileName = "/SaveData.rckt";

bool FileSerialiser::Init(bool usbActive)
{
  if (!LittleFS.begin(true))
  {
    OUTPUT_TEXT_ARES("LittleFS Mount Failed.");
    return false;
  }

  if (!usbActive)
  {
    m_File = LittleFS.open(s_SaveLogsFileName, FILE_APPEND);
    if (!m_File)
    {
      OUTPUT_TEXT_ARES("Failed to open file for writing [mode=appending].");
      return false;
    }
  }
  else
  {
    s_MSC.usbMSC.setID("MARS", "Flash", "1.0");
    s_MSC.usbMSC.setCapacity(MSC_DISK_BLOCK_COUNT, MSC_BLOCK_SIZE);
    s_MSC.usbMSC.setReadWriteCallback(MSC::ReadCallback, NULL, NULL);
    s_MSC.usbMSC.begin();

    OUTPUT_TEXT_ARES("Entered USB Mode...");
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

// void FileSerialiser::OutAll()
// {
//   fs::File file = LittleFS.open(s_SaveLogsFileName);
//   if (!file)
//   {
//     OUTPUT_TEXT_ARES("Failed to open file for reading.");
//     return;
//   }

//   SaveData data;
//   while (file.available() >= sizeof(SaveData))
//   {
//     size_t bytesRead = file.read((uint8_t*)&data, sizeof(SaveData));
    
//     #define OUTPUT_CSV_VEC(v) v.PrintRaw(); OUTPUT_TEXT_ARES(", ")
//     #define OUTPUT_CSV_FLOAT(v) OUTPUT_FLOAT_ARES(v, 3); OUTPUT_TEXT_ARES(", ")

//     OUTPUT_FLOAT_ARES(data.TimeStamp, 3);
//     OUTPUT_TEXT_ARES(", ");
//     // Current State
//     OUTPUT_CSV_VEC(data.CurrentState.Position);
//     OUTPUT_CSV_VEC(data.CurrentState.Velocity);
//     OUTPUT_CSV_VEC(data.CurrentState.Orientation);
//     OUTPUT_CSV_VEC(data.CurrentState.BiasMeanAccel);
//     OUTPUT_CSV_VEC(data.CurrentState.BiasMeanGyro);
//     OUTPUT_CSV_VEC(data.CurrentState.BiasDeltaAccel);
//     OUTPUT_CSV_VEC(data.CurrentState.BiasDeltaGyro);
//     // IMUs
//     OUTPUT_CSV_VEC(data.ProcessInputs.Accel1);
//     OUTPUT_CSV_VEC(data.ProcessInputs.Gyro1);
//     OUTPUT_CSV_VEC(data.ProcessInputs.Accel2);
//     OUTPUT_CSV_VEC(data.ProcessInputs.Gyro2);
//     // Sensors
//     OUTPUT_CSV_VEC(data.SensorReadings.GPS);
//     OUTPUT_CSV_VEC(data.SensorReadings.Mag);
//     OUTPUT_CSV_VEC(data.SensorReadings.DeltaAccel);
//     OUTPUT_CSV_VEC(data.SensorReadings.DeltaGyro);
// #if BAROMETER_ENABLED
//     OUTPUT_CSV_FLOAT(data.SensorReadings.Barom);
// #endif
//     // PID
//     OUTPUT_CSV_VEC(data.PID.Target);
//     OUTPUT_CSV_VEC(data.PID.TargetOffset);
//     OUTPUT_CSV_VEC(data.PID.TargetHeading);
//     OUTPUT_CSV_VEC(data.PID.ServoOrientation);
//     OUTPUT_CSV_FLOAT(data.PID.Thrust);
    
//     #undef OUTPUT_CSV_FLOAT
//     #undef OUTPUT_CSV_VEC

//     if (bytesRead != sizeof(SaveData))
//       OUTPUT_TEXT_ARES("File Write Error: Incomplete Data Written.");
//   }
// }

int32_t MSC::ReadCallback(uint32_t logicalBlockAddress, void* buffer, uint32_t bufferSize)
{
  if (logicalBlockAddress == 0) // FAT Boot Sector
  {
    memset(buffer, 0, bufferSize);
    // Insert faked boot-sectors so is detected as a valid disk
    uint8_t* buff = (uint8_t*)buffer;
    buff[510] = 0x55;
    buff[511] = 0xAA;
    return bufferSize;
  }

  fs::File flightLog = LittleFS.open(s_SaveLogsFileName);
  if (!flightLog)
  {
    memset(buffer, 0, bufferSize);
    return bufferSize;
  }

  uint32_t fileOffset = (logicalBlockAddress - 1) * MSC_BLOCK_SIZE;
  if (fileOffset < flightLog.size())
  {
    flightLog.seek(fileOffset);
    flightLog.read((uint8_t*)buffer, bufferSize);
  }
  else
  {
    memset(buffer, 0, bufferSize);
  }

  flightLog.close();
  return bufferSize;
}
