#include "RocketINS.h"

void RocketINS::UpdateGroundPreLaunch()
{
  m_SumAccelerationBMI += AccelBMI;
  m_SumAccelerationTDK += AccelTDK;

  m_CalibrationSamples++;

  if (m_CalibrationSamples >= 200)
  {
    Vector3 averageAccelBMI = m_SumAccelerationBMI * (1.0f / (float)m_CalibrationSamples);
    Vector3 averageAccelTDK = m_SumAccelerationTDK * (1.0f / (float)m_CalibrationSamples);
    if (averageAccelBMI.magSq() > 0.01f)
    {
      averageAccelBMI.normalise();
      float dot_g = -averageAccelBMI.z;
      float cross_x = averageAccelBMI.y;
      float cross_y = -averageAccelBMI.x;
      float s_g = sqrtf((1.0f + dot_g) * 2.0f);
      if (s_g > 0.001f)
      {
        Quaternion& orientation = EKF.CurrentState.Orientation;
        orientation.w = s_g * 0.5f;
        orientation.x = cross_x / s_g;
        orientation.y = cross_y / s_g;
        orientation.z = 0.0f;
      }

      if (averageAccelTDK.magSq() > 0.01f)
        m_SensorAlignmentTDK = Quaternion{ averageAccelTDK.normalised(), averageAccelBMI.normalised() };
    }
    
    m_CalibrationSamples = 0;
    m_SumAccelerationBMI *= 0.0f;
    m_SumAccelerationTDK *= 0.0f;
  }
}

void RocketINS::UpdateFlight(float delta)
{
  EKF.ProcessInputs.Gyro1 = GyroBMI;
  EKF.ProcessInputs.Accel1 = AccelBMI;
  EKF.ProcessInputs.Gyro2 = m_SensorAlignmentTDK.rotateVector(GyroTDK);
  EKF.ProcessInputs.Accel2 = m_SensorAlignmentTDK.rotateVector(AccelTDK);
  EKF.Predict(delta);

  // Other sensors will be checked if available and update the filter here...:
}
