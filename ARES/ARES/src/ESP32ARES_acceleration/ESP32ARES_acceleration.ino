#include "KalmanFilter.h"
#include "Sensors.h"

#include "SaveData.h"
#include "PID.h"

// Onboard WS2812 RGB Parameter
#define RGB_LED_PIN 38  // WS2812 LED Pin for N8R8

KalmanFilter KF;

enum class CalibrationStage {
  SETUP = 0,
  CALIBRATE_IMU_UP,
  CALIBRATE_IMU_DOWN,
  CALIBRATE_STATE,
  READY
};
CalibrationStage currentCalibrationStage = CalibrationStage::SETUP;

unsigned long lastMicros = 0;  // Shared clock for dt tracking

FileSerialiser FS{ "/SaveData.rckt" };
SaveDataBuffer dataBuffer;

void OUTPUT_TEXT_ARES(const char* txt) {
  Serial.print(txt);
}

void OUTPUT_FLOAT_ARES(float num, uint8_t dp) {
  Serial.println(num, dp);
}

void setup() {
  // Drive the onboard WS2812 LED green
  neopixelWrite(RGB_LED_PIN, 128, 0, 0);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F(" --- DIRECT AVIONICS MULTI-SENSOR ENGINE --- "));

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

  FS.Init();
  dataBuffer.Data[0].CurrentState = KF.CurrentState;
  dataBuffer.Data[0].ProcessInputs = KF.ProcessInputs;
  dataBuffer.Data[0].SensorReadings = KF.SensorReadings;
  dataBuffer.Data[0].PIDState.Target = Vector3{ 0.0f, 2.0f, 0.0f };
  dataBuffer.Data[0].PIDState.TargetOffset = Vector3{ 0.0f, 1.0f, 0.0f };
  dataBuffer.Data[0].PIDState.TargetHeading = Vector3{ 0.0f, 1.0f, 0.0f };
  dataBuffer.Data[0].PIDState.ServoOrientation = Vector2{ 0.0f, 0.0f };
  dataBuffer.Data[0].PIDState.Thrust = 10.0f;
  FS.Submit(dataBuffer);
  FS.Close();

  FS.OutAll();

  currentCalibrationStage = CalibrationStage::CALIBRATE_IMU_UP;
  lastMicros = micros();  // Establish system reference frame clock
}

uint16_t imuAvgFrame = 0;
Vector3 averageAcc1, averageAcc2;
Vector3 averageUpAcc1, averageUpAcc2;
void AverageReadingsIMU() {
  float scaleOld = (float)imuAvgFrame / (float)(imuAvgFrame + 1);
  float scaleNew = 1.0f - scaleOld;

  averageAcc1 = (averageAcc1 * scaleOld) + (KF.ProcessInputs.Accel1 * scaleNew);
  averageAcc2 = (averageAcc2 * scaleOld) + (KF.ProcessInputs.Accel2 * scaleNew);

  imuAvgFrame += 1;
}

void CalibrateUpIMU() {
  // Assuming 300Hz, spend 5s = ~1500frames
  if (imuAvgFrame < 1500) {
    if (imuAvgFrame == 0) {
      averageAcc1 *= 0.0f;
      averageAcc2 *= 0.0f;
      Serial.println(F("Calibrating IMUs Orientation..."));
      Serial.println(F("Averaging IMUs Up Acceleration... [Ensure Rocket is completely vertical]"));
    }

    AverageReadingsIMU();
    return;
  }

  averageUpAcc1 = averageAcc1;
  averageUpAcc2 = averageAcc2;
  Serial.println(F("SUCCESS: IMU Average Up Acceleration Measured"));
  Serial.println(F("Flip Rocket 180* within 10s"));
  currentCalibrationStage = CalibrationStage::CALIBRATE_IMU_DOWN;
  imuAvgFrame = 0;
}

void CalibrateDownIMU() {
  // Assuming 300Hz, spend 5s = ~1500frames
  if (imuAvgFrame < 1500) {
    if (imuAvgFrame == 0) {
      averageAcc1 *= 0.0f;
      averageAcc2 *= 0.0f;
      Serial.println(F("Averaging IMUs Down Acceleration... [Ensure Rocket is flipped 180*]"));
    }

    AverageReadingsIMU();
    return;
  }

  KF.CalibrateIMURotationalOffset(averageUpAcc1, averageAcc1, averageUpAcc2, averageAcc2);
  Serial.println(F("SUCCESS: IMU Rotational Offset Calibrated"));
  currentCalibrationStage = CalibrationStage::CALIBRATE_STATE;
  imuAvgFrame = 0;
}

bool CalibrateState() {
  // Assuming 300Hz, spend 5s = ~1500frames
  if (imuAvgFrame < 1500) {
    if (imuAvgFrame == 0) {
      averageAcc1 *= 0.0f;
      averageAcc2 *= 0.0f;
      Serial.println(F("Calibrating Kalman Filter State... "));
    }

    AverageReadingsIMU();
    return false;
  }

  KF.CalibrateInitialState(averageAcc1, averageAcc2);
  Serial.println(F("SUCCESS: Kalman Filter's Initial State Calibrated"));

  currentCalibrationStage = CalibrationStage::READY;
  imuAvgFrame = 0;
  return true;
}

void loop() {
  unsigned long currentMicros = micros();
  float dt = (float)(currentMicros - lastMicros) / 1000000.0f;
  lastMicros = currentMicros;
  if (dt <= 0.0f || dt > 0.5f)
    dt = 0.01f;  // Outlier guard filter

  IMUs::GetReadingsBMI(KF.ProcessInputs.Accel1, KF.ProcessInputs.Gyro1);
  IMUs::GetReadingsTDK(KF.ProcessInputs.Accel2, KF.ProcessInputs.Gyro2);
  KF.SensorReadings.DeltaAccel = KF.ProcessInputs.Accel1 - KF.ProcessInputs.Accel2;
  KF.SensorReadings.DeltaGyro = KF.ProcessInputs.Gyro1 - KF.ProcessInputs.Gyro2;

  Vector3 tdkGyroCopy = KF.ProcessInputs.Gyro2;

  if (currentCalibrationStage == CalibrationStage::CALIBRATE_IMU_UP) {
    CalibrateUpIMU();
    return;
  }

  static float accDelta = 0.0f;
  if (currentCalibrationStage == CalibrationStage::CALIBRATE_IMU_DOWN) {
    if (imuAvgFrame > 0 || accDelta > 10.0f)
      CalibrateDownIMU();

    accDelta += dt;
    return;
  }

  KF.CorrectIMUReadings();

  if (currentCalibrationStage == CalibrationStage::CALIBRATE_STATE)
    CalibrateState();

  static float accDeltaCalibrationTest = 0.0f;
  accDeltaCalibrationTest += dt;
  if (accDeltaCalibrationTest > 10.0f) {
    if (imuAvgFrame == 0)
      Serial.println("Recalibrating State [Every 15 Seconds]...");

    if (CalibrateState()) {
      accDeltaCalibrationTest = 0.0f;
      Serial.println("State Recalibrated.");
    }
  }

  KF.Predict(dt);
  KF.UpdateDeltaAccel();
  KF.UpdateDeltaGyro();

  // THE [OLD] QUATERNION STUFF

  static Quaternion tdkOrient;

  float speed = tdkGyroCopy.mag() * dt;
  if (speed > 0.000001f) {
    Vector3 axis = tdkGyroCopy.normalised();
    Quaternion deltaOrientation{ axis * std::sin(speed * 0.5f), std::cos(speed * 0.5f) };
    tdkOrient = (tdkOrient * deltaOrientation).normalised();
  }

  float tdkRoll = atan2(2.0f * (tdkOrient.w * tdkOrient.x + tdkOrient.y * tdkOrient.z), 1.0f - 2.0f * (tdkOrient.x * tdkOrient.x + tdkOrient.y * tdkOrient.y)) * (180.0f / PI);
  float tdkPitch = asin(fmax(-1.0f, fmin(1.0f, 2.0f * (tdkOrient.w * tdkOrient.y - tdkOrient.z * tdkOrient.x)))) * (180.0f / PI);
  float tdkYaw = atan2(2.0f * (tdkOrient.w * tdkOrient.z + tdkOrient.x * tdkOrient.y), 1.0f - 2.0f * (tdkOrient.y * tdkOrient.y + tdkOrient.z * tdkOrient.z)) * (180.0f / PI);

  // float dT_dQW = 0.5f * (-tdkQX * tdkRadX - tdkQY * tdkRadY - tdkQZ * tdkRadZ) * dt;
  // float dT_dQX = 0.5f * ( tdkQW * tdkRadX + tdkQY * tdkRadZ - tdkQZ * tdkRadY) * dt;
  // float dT_dQY = 0.5f * ( tdkQW * tdkRadY - tdkQX * tdkRadZ + tdkQZ * tdkRadX) * dt;
  // float dT_dQZ = 0.5f * ( tdkQW * tdkRadZ + tdkQX * tdkRadY - tdkQY * tdkRadX) * dt;
  // tdkQW += dT_dQW; tdkQX += dT_dQX; tdkQY += dT_dQY; tdkQZ += dT_dQZ;
  // float tdkNorm = sqrt(tdkQW * tdkQW + tdkQX * tdkQX + tdkQY * tdkQY + tdkQZ * tdkQZ);
  // if (tdkNorm > 0.0f) { tdkQW /= tdkNorm; tdkQX /= tdkNorm; tdkQY /= tdkNorm; tdkQZ /= tdkNorm; }
  // float tdkRoll  = atan2(2.0f * (tdkQW * tdkQX + tdkQY * tdkQZ), 1.0f - 2.0f * (tdkQX * tdkQX + tdkQY * tdkQY)) * (180.0f / PI);
  // float tdkPitch = asin(fmax(-1.0f, fmin(1.0f, 2.0f * (tdkQW * tdkQY - tdkQZ * tdkQX)))) * (180.0f / PI);
  // float tdkYaw   = atan2(2.0f * (tdkQW * tdkQZ + tdkQX * tdkQY), 1.0f - 2.0f * (tdkQY * tdkQY + tdkQZ * tdkQZ)) * (180.0f / PI);

  // TELEMETRY OUTPUT ENGINE (Serial Stream)

  static float accDeltaLog = 0.0f;
  accDeltaLog += dt;
  if (accDeltaLog > 1.000f)  // Output only once every 1000ms (1s)
  {
    accDeltaLog = 0.0f;

    Serial.print(F("  POSITION XYZ:"));
    Serial.print(KF.CurrentState.Position.x, 1);
    Serial.print(F(","));
    Serial.print(KF.CurrentState.Position.y, 1);
    Serial.print(F(","));
    Serial.print(KF.CurrentState.Position.z, 1);

    Serial.print(F("  ORI XYZ:"));
    Vector3 up = KF.CurrentState.Orientation.rotateVector(Vector3(0.0, 1.0, 0.0));
    Serial.print(up.x, 4);
    Serial.print(F(","));
    Serial.print(up.y, 4);
    Serial.print(F(","));
    Serial.print(up.z, 4);

    Serial.print(F("  VELOCITY XYZ:"));
    Serial.print(KF.CurrentState.Velocity.x, 1);
    Serial.print(F(","));
    Serial.print(KF.CurrentState.Velocity.y, 1);
    Serial.print(F(","));
    Serial.print(KF.CurrentState.Velocity.z, 1);

    Serial.print(F(" | TDK_EULER[R,P,Y]:"));
    Serial.print(tdkRoll, 1);
    Serial.print(F(","));
    Serial.print(tdkPitch, 1);
    Serial.print(F(","));
    Serial.print(tdkYaw, 1);

    Serial.print(F("  |  TDK [m/s² X,Y,Z]: "));
    Serial.print(KF.ProcessInputs.Accel2.x, 3);
    Serial.print(F(", "));
    Serial.print(KF.ProcessInputs.Accel2.y, 3);
    Serial.print(F(", "));
    Serial.print(KF.ProcessInputs.Accel2.z, 3);

    Serial.println();
  }

  delay(3);
}
