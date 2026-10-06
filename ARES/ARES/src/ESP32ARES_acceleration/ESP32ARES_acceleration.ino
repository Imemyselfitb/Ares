#include "RocketINS.h"
#include "Sensors.h"

#include "SaveData.h"
#include "PID.h"

#include <tusb.h>

// Onboard WS2812 RGB Parameter
#define RGB_LED_PIN 38  // WS2812 LED Pin for N8R8

const unsigned long LOOP_TIME_US = 5000;

RocketINS INS;
bool isLaunched = false;

unsigned long lastUpdate = 0;

FileSerialiser FS;
SaveDataBuffer dataBuffer;

bool isInFlightMode = false;

void OUTPUT_TEXT_ARES(const char* txt) {
  Serial.print(txt);
}

void OUTPUT_FLOAT_ARES(float num, uint8_t dp) {
  Serial.print(num, dp);
}

void setup() {
  // Drive the onboard WS2812 LED green
  neopixelWrite(RGB_LED_PIN, 128, 0, 0);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F(" --- DIRECT AVIONICS MULTI-SENSOR ENGINE --- "));

  isInFlightMode = tud_mounted();

  // Initialize I2C bus channels
  Serial.println(F("Booting Bosch BMI088... "));
  int statusCodeBMI = IMUs::BootBMI();
  if (statusCodeBMI == 0)
    Serial.println(F("SUCCESS: Bosch BMI088 configured to [24G / 2000DPS]"));
  else {
    Serial.print(F("FAILED. Error: "));
    Serial.println(statusCodeBMI);
  }

  Serial.println(F("Booting TDK ICM-45686... "));
  int statusCodeTDK = IMUs::BootTDK();
  if (statusCodeTDK == 0)
    Serial.println(F("SUCCESS: TDK ICM-45686 configured to [32G / 2000DPS]"));
  else {
    Serial.print(F("FAILED. Error: "));
    Serial.println(statusCodeTDK);
  }

  Serial.println(F("Booting BMM350... "));
  bool successfulBMM = Magnetometer::Init();
  if (successfulBMM)
    Serial.println(F("SUCCESS: BMM350 initialised!"));
  else
    Serial.println("FAILED.");

  // FS.Init();
  // dataBuffer.Data[0].CurrentState = KF.CurrentState;
  // dataBuffer.Data[0].ProcessInputs = KF.ProcessInputs;
  // dataBuffer.Data[0].SensorReadings = KF.SensorReadings;
  // dataBuffer.Data[0].PIDState.Target = Vector3{ 0.0f, 2.0f, 0.0f };
  // dataBuffer.Data[0].PIDState.TargetOffset = Vector3{ 0.0f, 1.0f, 0.0f };
  // dataBuffer.Data[0].PIDState.TargetHeading = Vector3{ 0.0f, 1.0f, 0.0f };
  // dataBuffer.Data[0].PIDState.ServoOrientation = Vector2{ 0.0f, 0.0f };
  // dataBuffer.Data[0].PIDState.Thrust = 10.0f;
  // FS.Submit(dataBuffer);
  // FS.Close();

  // FS.OutAll();

  // FS.Init(!isInFlightMode);
  // if (isInFlightMode)
  // {
  //   FS.Close(!isInFlightMode);
  // }

  lastUpdate = micros();  // Establish system reference frame clock
}

void loop() {
  while ((micros() - lastUpdate) < LOOP_TIME_US)
  {
    delay(1);
    if (Serial.available() > 0)
    {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("launch"))
      {
        isLaunched = true;
        Serial.println(F("[SYSTEM] !!! LAUNCH COMMAND RECORDED. LOCKING POSTURE SIGNATURES !!!"));
      }
    }
  }

  unsigned long now = micros();
  float dt = (float)(now - lastUpdate) / 1000000.0f;
  lastUpdate = now;
  if (dt <= 0.0f || dt > 0.5f)
    dt = 0.01f;  // Outlier guard filter

  IMUs::GetReadingsBMI(INS.AccelBMI, INS.GyroBMI);
  IMUs::GetReadingsTDK(INS.AccelTDK, INS.GyroTDK);
  if (!isLaunched)
  {
    INS.UpdateGroundPreLaunch();
  }
  else
  {
    INS.UpdateFlight(dt);
  }
  
  // TELEMETRY OUTPUT ENGINE (Serial Stream)

  static float accDeltaLog = 0.0f;
  accDeltaLog += dt * 100000.0f;
  if (accDeltaLog > 0.500f)  // Output only once every 1000ms (1s)
  {
    accDeltaLog = 0.0f;

    Serial.print(F("  ORI XYZ:"));
    Vector3 up = INS.EKF.CurrentState.Orientation.rotateVector(Vector3{ 0.0, 1.0, 0.0 });
    Serial.print(up.x, 4);
    Serial.print(F(","));
    Serial.print(up.y, 4);
    Serial.print(F(","));
    Serial.print(up.z, 4);

    const Vector3& pos = INS.EKF.CurrentState.Position;
    Serial.print(F("  POSITION XYZ:"));
    Serial.print(pos.x, 1);
    Serial.print(F(","));
    Serial.print(pos.y, 1);
    Serial.print(F(","));
    Serial.print(pos.z, 1);

    const Vector3& vel = INS.EKF.CurrentState.Velocity;
    Serial.print(F("  VELOCITY XYZ:"));
    Serial.print(vel.x, 1);
    Serial.print(F(","));
    Serial.print(vel.y, 1);
    Serial.print(F(","));
    Serial.print(vel.z, 1);

    Serial.print(F("  |  TDK [m/s² X,Y,Z]: "));
    Serial.print(INS.AccelTDK.x, 3);
    Serial.print(F(", "));
    Serial.print(INS.AccelTDK.y, 3);
    Serial.print(F(", "));
    Serial.print(INS.AccelTDK.z, 3);

    Vector3 mag;
    if (Magnetometer::GetReading(mag))
    {
      Serial.print(F("  |  MAGNET: [X,Y,Z]: "));
      mag.Print();
    }
    else
    {
      Serial.print(F("  |  MAGNET FAILED READING!"));
    }

    Serial.println();
  }
}
