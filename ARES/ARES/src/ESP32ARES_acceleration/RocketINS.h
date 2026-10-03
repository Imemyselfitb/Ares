#pragma once

#include "KalmanFilter.h"

class RocketINS
{
public:
  RocketINS() {}

  void UpdateGroundPreLaunch();
  void UpdateFlight(float delta);

public:
  KalmanFilter EKF;
  Vector3 AccelTDK;
  Vector3 AccelBMI;
  Vector3 GyroTDK;
  Vector3 GyroBMI;

private:
  Quaternion m_SensorAlignmentTDK;

  Vector3 m_SumAccelerationTDK;
  Vector3 m_SumAccelerationBMI;
  uint32_t m_CalibrationSamples;
};
