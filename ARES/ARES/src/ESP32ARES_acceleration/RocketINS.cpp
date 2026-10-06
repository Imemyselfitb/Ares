#include "RocketINS.h"

void RocketINS::UpdateGroundPreLaunch()
{
  m_SumAccelerationBMI += AccelBMI;
  m_SumAccelerationTDK += AccelTDK;

  m_CalibrationSamples++;

  if (m_CalibrationSamples >= 200)
  {
    Vector3 averageAccelBMI = m_SumAccelerationBMI * (1.0f / 200.0f);
    Vector3 averageAccelTDK = m_SumAccelerationTDK * (1.0f / 200.0f);
    if (averageAccelBMI.magSq() > 0.01f)
    {
      averageAccelBMI.normalise();
      EKF.CurrentState.Orientation = Quaternion{ averageAccelBMI, Vector3{ 0.0f, 1.0f, 0.0f } };
      if (averageAccelTDK.magSq() > 0.01f)
        m_SensorAlignmentTDK = Quaternion{ averageAccelTDK.normalised(), averageAccelBMI };
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
