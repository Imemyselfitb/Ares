#include "KalmanFilter.h"

// Onboard WS2812 RGB Parameter 
#define RGB_LED_PIN 38  // WS2812 LED Pin for N8R8

KalmanFilter KF;

unsigned long lastMicros = 0; // Shared clock for dt tracking

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
    Serial.print(F("Booting Bosch BMI088... "));
    int statusCodeBMI = IMUs::BootBMI();
    if (statusCodeBMI == 0)
        Serial.println(F("SUCCESS: Bosch BMI088 configured to [24G / 2000DPS]"));
    else
    {
        Serial.print(F("FAILED. Error: "));
        Serial.println(statusCodeTDK);
    }

    Serial.print(F("Booting TDK ICM-45686... "));
    int statusCodeTDK = IMU::BootTDK();
    if (statusCodeTDK == 0)
        Serial.println(F("SUCCESS: TDK ICM-45686 configured to [32G / 2000DPS]"));
    else
    {
        Serial.print(F("FAILED. Error: "));
        Serial.println(statusCodeTDK);
    }
    
    IMUs::GetReadingsBMI(KF.ProcessInputs.Accel1, KF.ProcessInputs.Gyro1);
    IMUs::GetReadingsTDK(KF.ProcessInputs.Accel2, KF.ProcessInputs.Gyro2);
    KF.CalibrateInitialState();

    lastMicros = micros(); // Establish system reference frame clock
}

void loop()
{
    unsigned long currentMicros = micros();
    float dt = (float)(currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;
    if (dt <= 0.0f || dt > 0.5f)
        dt = 0.01f; // Outlier guard filter

    IMUs::GetReadingsBMI(KF.ProcessInputs.Accel1, KF.ProcessInputs.Gyro1);
    IMUs::GetReadingsTDK(KF.ProcessInputs.Accel2, KF.ProcessInputs.Gyro2);

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
