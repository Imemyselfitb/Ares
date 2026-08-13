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
ICM456xx icm{ Wire, 1 }; 

KalmanFilter KF;

unsigned long lastMicros = 0; // Shared clock for dt tracking

// I2C write transaction utility
void writeReg(uint8_t dev, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true); // Always force standard bus termination on writes
}

void OUTPUT_TEXT_ARES(const char* txt)
{
    Serial.println(txt);
}

void OUTPUT_FLOAT_ARES(float num, uint8_t dp)
{
    Serial.println(num, dp);
}

void setup()
{
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
    if (icmStatus != 0)
    {
        Serial.print(F("FAILED. Error: "));
        Serial.println(icmStatus);
    }
    else
    {
        icm.startAccel(100, 32);   // Start at 100Hz ODR and +/- 32G range
        icm.startGyro(100, 2000);  // Wake the integrated TDK gyro engine at 100Hz and +/- 2000 DPS
        Serial.println(F("SUCCESS (32G / 2000DPS Active)"));
    }

    getReadingsIMU_BMI(KF.ProcessInputs.Accel1, KF.ProcessInputs.Gyro1);
    getReadingsIMU_TDK(KF.ProcessInputs.Accel2, KF.ProcessInputs.Gyro2);
    KF.CalibrateInitialState();

    lastMicros = micros(); // Establish system reference frame clock
}

void getReadingsIMU_BMI(Vector3& outAccel, Vector3& outGyro)
{
    Wire.beginTransmission(BMI088_ACCEL_ADDR);
    Wire.write(0x12); 
    if (Wire.endTransmission(false) == 0)
    {
        Wire.requestFrom((uint8_t)BMI088_ACCEL_ADDR, (uint8_t)6);
        if (Wire.available() >= 6)
        {
            int16_t ax = (int16_t)(Wire.read() | (Wire.read() << 8));
            int16_t ay = (int16_t)(Wire.read() | (Wire.read() << 8));
            int16_t az = (int16_t)(Wire.read() | (Wire.read() << 8));
            
            outAccel.x = ((float)ax * 24.0f / 32768.0f) * 9.80665f;
            outAccel.y = ((float)ay * 24.0f / 32768.0f) * 9.80665f;
            outAccel.z = ((float)az * 24.0f / 32768.0f) * 9.80665f;
        }
    }
    
    Wire.beginTransmission(BMI088_GYRO_ADDR);
    Wire.write(0x02); // Gyro Rate X LSB Register Address
    if (Wire.endTransmission(false) == 0)
    {
        Wire.requestFrom((uint8_t)BMI088_GYRO_ADDR, (uint8_t)6);
        if (Wire.available() >= 6)
        {
            int16_t gx = (int16_t)(Wire.read() | (Wire.read() << 8));
            int16_t gy = (int16_t)(Wire.read() | (Wire.read() << 8));
            int16_t gz = (int16_t)(Wire.read() | (Wire.read() << 8));
            
            outGyro.x = ((float)gx * 2000.0f / 32768.0f) * (PI / 180.0f);
            outGyro.y = ((float)gy * 2000.0f / 32768.0f) * (PI / 180.0f);
            outGyro.z = ((float)gz * 2000.0f / 32768.0f) * (PI / 180.0f);
        }
    }
}

void getReadingsIMU_TDK(Vector3& outAccel, Vector3& outGyro)
{
    inv_imu_sensor_data_t icmData; 
    icm.getDataFromRegisters(icmData);

    outAccel.x = ((float)icmData.accel_data[0] * 32.0f / 32768.0f) * 9.80665f;
    outAccel.y = ((float)icmData.accel_data[1] * 32.0f / 32768.0f) * 9.80665f;
    outAccel.z = ((float)icmData.accel_data[2] * 32.0f / 32768.0f) * 9.80665f;
    
    outGyro.x = ((float)icmData.gyro_data[0] * 2000.0f / 32768.0f) * (PI / 180.0f);
    outGyro.y = ((float)icmData.gyro_data[1] * 2000.0f / 32768.0f) * (PI / 180.0f);
    outGyro.z = ((float)icmData.gyro_data[2] * 2000.0f / 32768.0f) * (PI / 180.0f);
}

void loop()
{
    unsigned long currentMicros = micros();
    float dt = (float)(currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;
    if (dt <= 0.0f || dt > 0.5f)
        dt = 0.01f; // Outlier guard filter

    getReadingsIMU_BMI(KF.ProcessInputs.Accel1, KF.ProcessInputs.Gyro1);
    getReadingsIMU_TDK(KF.ProcessInputs.Accel2, KF.ProcessInputs.Gyro2);

    KF.SensorReadings.DeltaAccel = KF.ProcessInputs.Accel1 - KF.ProcessInputs.Accel2;
    KF.SensorReadings.DeltaGyro = KF.ProcessInputs.Gyro1 - KF.ProcessInputs.Gyro2; 
    KF.Predict(dt);

    KF.UpdateDeltaAccel();
    KF.UpdateDeltaGyro();

    // TELEMETRY OUTPUT ENGINE (Serial Stream)

    static float accDelta = 0.0f;
    accDelta += dt;
    if (accDelta > 0.100) // Output only once every 100ms
    {
        Serial.print(F("  POSITION XYZ:")) ;
        Serial.print(KF.CurrentState.Position.x, 1); Serial.print(F(","));
        Serial.print(KF.CurrentState.Position.y, 1); Serial.print(F(","));
        Serial.print(KF.CurrentState.Position.z, 1);

        Serial.print(F("  ORI XYZ:"));
        Vector3 up = KF.CurrentState.Orientation.rotateVector(Vector3(0.0, 1.0, 0.0));
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
    }

    delay(3);
}
