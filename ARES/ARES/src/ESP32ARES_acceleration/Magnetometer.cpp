#include <Sensors.h>

#include <BMM350.h>

#define I2C_SDA 4
#define I2C_SCL 5

static BMM350 s_Magnetometer{ 0x14 };

bool Magnetometer::Init()
{
  if (!g_IsWireInitialised)
  {
      Wire.begin(I2C_SDA, I2C_SCL);
      Wire.setClock(100000); // 100kHz for noise immunity
      delay(400);
      g_IsWireInitialised = true;
  }

  bool successful = s_Magnetometer.begin(&Wire);
  return successful;
}

bool Magnetometer::GetReading(Vector3 &outDirection)
{
  bool successful = s_Magnetometer.readMagnetometerData(outDirection.x, outDirection.y, outDirection.z);
  return successful;
}
