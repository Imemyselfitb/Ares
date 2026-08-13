#include <Wire.h>
#include <ICM45686.h> // TDK Balanced-Gyro Driver
#include <KalmanFilter.h>

#define I2C_SDA 4
#define I2C_SCL 5

// Onboard WS2812 RGB Parameter 
#define RGB_LED_PIN 38  // WS2812 LED Pin for N8R8

// Hardware Addresses 
#define BMI088_ACCEL_ADDR 0x18
#define BMI088_GYRO_ADDR  0x68  // Bosch Gyro Core (SDO2 pulled low)
#define ICM456_ADDR       0x69

// Instantiate official TDK constructor interface targeting standard I2C bus
ICM456xx icm(Wire, 1); 

  KalmanFilter KF;

// QUATERNION ORIENTATION FIELDS

// TDK Orientation Tracking
float tdkQW = 1.0f, tdkQX = 0.0f, tdkQY = 0.0f, tdkQZ = 0.0f;

// Bosch Orientation Tracking
float bmiQW = 1.0f, bmiQX = 0.0f, bmiQY = 0.0f, bmiQZ = 0.0f;

unsigned long lastMicros = 0; // Shared clock for dt tracking

// I2C write transaction utility
void writeReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true); // Always force standard bus termination on writes
}

void OUTPUT_TEXT_ARES(const char* txt) {
  Serial.println(txt);
}

void OUTPUT_FLOAT_ARES(float num, uint8_t dp) {
  Serial.println(num, dp);
}

void setup() {
  // Drive the onboard WS2812 LED green
  neopixelWrite(RGB_LED_PIN, 128, 0, 0); 

  Serial.begin(115200);
  while(!Serial) delay(10);
  
  Serial.println(F(" --- DIRECT AVIONICS MULTI-SENSOR ENGINE --- "));
  
  // Initialize I2C bus channels
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100kHz for noise immunity
  delay(400);
  
  // 1. BOOT BOSCH BMI088 ACCELEROMETER 
  writeReg(BMI088_ACCEL_ADDR, 0x7E, 0xB6); // Step A: Soft-reset the sensor to clear error states
  delay(50);                               // Mandatory delay after soft-reset
  writeReg(BMI088_ACCEL_ADDR, 0x7C, 0x00); // Step B: Set ACC_PWR_CONF to active mode (turns on sensor)
  delay(20);
  writeReg(BMI088_ACCEL_ADDR, 0x7D, 0x04); // Step C: Set ACC_PWR_CTRL to 0x04 to fully wake internal sensor loops
  delay(50);                               // Wait for internal power rails to settle
  writeReg(BMI088_ACCEL_ADDR, 0x41, 0x03); // Step D: Set ACC_RANGE to +/- 24G rocket scale
  delay(10);
  writeReg(BMI088_ACCEL_ADDR, 0x40, 0xAC); // Step E: Configure ODR to 400Hz, Normal filter mode (0x2C | 0x80)
  delay(100);                              // Aerospace settling pause for stable data registers
  Serial.println(F("[OK] Bosch BMI088 Accel configured to 24G Rocket Scale."));
  
  // 2. BOOT BOSCH BMI088 GYROSCOPE (WAKE UP CORE)
  writeReg(BMI088_GYRO_ADDR, 0x14, 0xB6);  // Soft-reset the gyro register tables
  delay(50);
  writeReg(BMI088_GYRO_ADDR, 0x0F, 0x00);  // Set GYRO_RANGE to +/- 2000 DPS (Full Rocket Scale)
  delay(10);
  writeReg(BMI088_GYRO_ADDR, 0x10, 0x80);  // Set GYRO_BANDWIDTH to ODR 2000Hz / Filter BW 532Hz
  delay(10);
  writeReg(BMI088_GYRO_ADDR, 0x11, 0x00);  // Set GYRO_LPM1 to 0x00 (Take out of suspend -> Normal Mode)
  delay(100);                              // Give core state machine time to settle
  Serial.println(F("[OK] Bosch BMI088 Gyro configured to 2000 DPS Scale."));

  // 3. BOOT TDK ICM-45686 VIA THE OFFICIAL ENGINE 

  Serial.print(F("Booting TDK ICM-45686... "));
  int icmStatus = icm.begin();
  if (icmStatus != 0) {
    Serial.print(F("FAILED. Error: "));
    Serial.println(icmStatus);
  } else {
    icm.startAccel(100, 32);   // Start at 100Hz ODR and +/- 32G range
    icm.startGyro(100, 2000);  // Wake the integrated TDK gyro engine at 100Hz and +/- 2000 DPS
    Serial.println(F("SUCCESS (32G / 2000DPS Active)"));
  }
  
  lastMicros = micros(); // Establish system reference frame clock
}

void loop() {
  float bmiMS2_X = 0, bmiMS2_Y = 0, bmiMS2_Z = 0;
  float tdkMS2_X = 0, tdkMS2_Y = 0, tdkMS2_Z = 0;
  
  int16_t bGyroRawX = 0, bGyroRawY = 0, bGyroRawZ = 0;

  // Read & Align Bosch BMI088 Accel
  
  Wire.beginTransmission(BMI088_ACCEL_ADDR);
  Wire.write(0x12); 
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)BMI088_ACCEL_ADDR, (uint8_t)6);
    if (Wire.available() >= 6) {
      uint8_t xL = Wire.read(); uint8_t xH = Wire.read();
      uint8_t yL = Wire.read(); uint8_t yH = Wire.read();
      uint8_t zL = Wire.read(); uint8_t zH = Wire.read();
      
      int16_t bRawX = (int16_t)(xL | (xH << 8));
      int16_t bRawY = (int16_t)(yL | (yH << 8));
      int16_t bRawZ = (int16_t)(zL | (zH << 8));
      
      bmiMS2_X = ((float)bRawX * 24.0f / 32768.0f) * 9.80665f;
      bmiMS2_Y = ((float)bRawY * 24.0f / 32768.0f) * 9.80665f;
      bmiMS2_Z = ((float)bRawZ * 24.0f / 32768.0f) * 9.80665f;
    }
  }

  // READ BOSCH BMI088 GYROSCOPE CORE
  
  Wire.beginTransmission(BMI088_GYRO_ADDR);
  Wire.write(0x02); // Gyro Rate X LSB Register Address
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)BMI088_GYRO_ADDR, (uint8_t)6);
    if (Wire.available() >= 6) {
      uint8_t gxL = Wire.read(); uint8_t gxH = Wire.read();
      uint8_t gyL = Wire.read(); uint8_t gyH = Wire.read();
      uint8_t gzL = Wire.read(); uint8_t gzH = Wire.read();
      
      bGyroRawX = (int16_t)(gxL | (gxH << 8));
      bGyroRawY = (int16_t)(gyL | (gyH << 8));
      bGyroRawZ = (int16_t)(gzL | (gzH << 8));
    }
  }

  
  // Read & Scale TDK ICM-45686 Data
  
  inv_imu_sensor_data_t icmData; 
  icm.getDataFromRegisters(icmData);

  // Fixed indexing: reads index values 
  tdkMS2_X = ((float)icmData.accel_data[0] * 32.0f / 32768.0f) * 9.80665f;
  tdkMS2_Y = ((float)icmData.accel_data[1] * 32.0f / 32768.0f) * 9.80665f;
  tdkMS2_Z = ((float)icmData.accel_data[2] * 32.0f / 32768.0f) * 9.80665f;

  
  // CALCULATIONS MATRIX: Delta Time (dt) Tracking
  
  unsigned long currentMicros = micros();
  float dt = (float)(currentMicros - lastMicros) / 1000000.0f;
  lastMicros = currentMicros;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f; // Outlier guard filter

  
  // Process TDK Gyro into Quaternion & Euler
  
  // Fixed indexing: reads index values
  float tdkRadX = ((float)icmData.gyro_data[0] * 2000.0f / 32768.0f) * (PI / 180.0f);
  float tdkRadY = ((float)icmData.gyro_data[1] * 2000.0f / 32768.0f) * (PI / 180.0f);
  float tdkRadZ = ((float)icmData.gyro_data[2] * 2000.0f / 32768.0f) * (PI / 180.0f);

  float dT_dQW = 0.5f * (-tdkQX * tdkRadX - tdkQY * tdkRadY - tdkQZ * tdkRadZ) * dt;
  float dT_dQX = 0.5f * ( tdkQW * tdkRadX + tdkQY * tdkRadZ - tdkQZ * tdkRadY) * dt;
  float dT_dQY = 0.5f * ( tdkQW * tdkRadY - tdkQX * tdkRadZ + tdkQZ * tdkRadX) * dt;
  float dT_dQZ = 0.5f * ( tdkQW * tdkRadZ + tdkQX * tdkRadY - tdkQY * tdkRadX) * dt;

  tdkQW += dT_dQW; tdkQX += dT_dQX; tdkQY += dT_dQY; tdkQZ += dT_dQZ;
  float tdkNorm = sqrt(tdkQW * tdkQW + tdkQX * tdkQX + tdkQY * tdkQY + tdkQZ * tdkQZ);
  if (tdkNorm > 0.0f) { tdkQW /= tdkNorm; tdkQX /= tdkNorm; tdkQY /= tdkNorm; tdkQZ /= tdkNorm; }

  float tdkRoll  = atan2(2.0f * (tdkQW * tdkQX + tdkQY * tdkQZ), 1.0f - 2.0f * (tdkQX * tdkQX + tdkQY * tdkQY)) * (180.0f / PI);
  float tdkPitch = asin(fmax(-1.0f, fmin(1.0f, 2.0f * (tdkQW * tdkQY - tdkQZ * tdkQX)))) * (180.0f / PI);
  float tdkYaw   = atan2(2.0f * (tdkQW * tdkQZ + tdkQX * tdkQY), 1.0f - 2.0f * (tdkQY * tdkQY + tdkQZ * tdkQZ)) * (180.0f / PI);
  
  // Process Bosch Gyro into Quaternion & Euler
  
  float bRadX = ((float)bGyroRawX * 2000.0f / 32768.0f) * (PI / 180.0f);
  float bRadY = ((float)bGyroRawY * 2000.0f / 32768.0f) * (PI / 180.0f);
  float bRadZ = ((float)bGyroRawZ * 2000.0f / 32768.0f) * (PI / 180.0f);

  float dB_dQW = 0.5f * (-bmiQX * bRadX - bmiQY * bRadY - bmiQZ * bRadZ) * dt;
  float dB_dQX = 0.5f * ( bmiQW * bRadX + bmiQY * bRadZ - bmiQZ * bRadY) * dt;
  float dB_dQY = 0.5f * ( bmiQW * bRadY - bmiQX * bRadZ + bmiQZ * bRadX) * dt;
  float dB_dQZ = 0.5f * ( bmiQW * bRadZ + bmiQX * bRadY - bmiQY * bRadX) * dt;

  bmiQW += dB_dQW; bmiQX += dB_dQX; bmiQY += dB_dQY; bmiQZ += dB_dQZ;
  float bmiNorm = sqrt(bmiQW * bmiQW + bmiQX * bmiQX + bmiQY * bmiQY + bmiQZ * bmiQZ);
  if (bmiNorm > 0.0f) { bmiQW /= bmiNorm; bmiQX /= bmiNorm; bmiQY /= bmiNorm; bmiQZ /= bmiNorm; }

  float bmiRoll  = atan2(2.0f * (bmiQW * bmiQX + bmiQY * bmiQZ), 1.0f - 2.0f * (bmiQX * bmiQX + bmiQY * bmiQY)) * (180.0f / PI);
  float bmiPitch = asin(fmax(-1.0f, fmin(1.0f, 2.0f * (bmiQW * bmiQY - bmiQZ * bmiQX)))) * (180.0f / PI);
  float bmiYaw   = atan2(2.0f * (bmiQW * bmiQZ + bmiQX * bmiQY), 1.0f - 2.0f * (bmiQY * bmiQY + bmiQZ * bmiQZ)) * (180.0f / PI);
  
  KF.ProcessInputs.Accel1 = Vector3{ bmiMS2_X, bmiMS2_Y, bmiMS2_Z };
  KF.ProcessInputs.Gyro1 = Vector3{ bRadX, bRadY, bRadZ };
  KF.ProcessInputs.Accel2 = Vector3{ tdkMS2_X, tdkMS2_Y, tdkMS2_Z };
  KF.ProcessInputs.Gyro2 = Vector3{ tdkRadX, tdkRadY, tdkRadZ };

  KF.SensorReadings.DeltaAccel = KF.ProcessInputs.Accel1 - KF.ProcessInputs.Accel2;
  KF.SensorReadings.DeltaGyro = KF.ProcessInputs.Gyro1 - KF.ProcessInputs.Gyro2; 
  KF.Predict(dt);

  KF.UpdateDeltaAccel();
  KF.UpdateDeltaGyro();

  Vector3 up = KF.CurrentState.Orientation.rotateVector(Vector3(0.0, 1.0, 0.0));
  // TELEMETRY OUTPUT ENGINE (Serial Stream)

  Serial.print(F("  POSITION XYZ:")) ;
  Serial.print(KF.CurrentState.Position.x, 1); Serial.print(F(","));
  Serial.print(KF.CurrentState.Position.y, 1); Serial.print(F(","));
  Serial.print(KF.CurrentState.Position.z, 1);

  Serial.print(F("  ORI XYZ:"));
  Serial.print(up.x, 4);  Serial.print(F(","));
  Serial.print(up.y, 4); Serial.print(F(","));
  Serial.print(up.z, 4);

  Serial.print(F("  VELOCITY XYZ:"));
  Serial.print(KF.CurrentState.Velocity.x, 1); Serial.print(F(","));
  Serial.print(KF.CurrentState.Velocity.y, 1); Serial.print(F(","));
  Serial.print(KF.CurrentState.Velocity.z, 1);

  Serial.print(F(" | TDK_EULER[R,P,Y]:"));
  Serial.print(tdkRoll, 1);  Serial.print(F(","));
  Serial.print(tdkPitch, 1); Serial.print(F(","));
  Serial.print(tdkYaw, 1);

  Serial.print(F("  |  TDK [m/s² X,Y,Z]: "));
  Serial.print(tdkMS2_X, 3); Serial.print(F(", "));
  Serial.print(tdkMS2_Y, 3); Serial.print(F(", "));
  Serial.print(tdkMS2_Z, 3);
  
  Serial.println();

  // 10Hz Telemetry Pacing Delay
  delay(3);
  }