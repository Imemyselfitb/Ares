#pragma once

#include <cmath>
#include "Matrix.h"
#include "Vector3.h"
#include "Quaternion.h"

#define BAROMETER_ENABLED 0

struct KalmanFilterState
{
	Vector3 Position{};
	Vector3 Velocity{};
	Quaternion Orientation{};
	Vector3 BiasMeanAccel{};
	Vector3 BiasMeanGyro{};
	Vector3 BiasDeltaAccel{};
	Vector3 BiasDeltaGyro{};
};

struct KalmanFilterProcessInputs
{
	Vector3 Accel1{};
	Vector3 Gyro1{};
	Vector3 Accel2{};
	Vector3 Gyro2{};
};

struct KalmanFilterSensorReadings
{
	float Barom;
	Vector3 GPS;
	Vector3 Mag;
	Vector3 DeltaAccel;
	Vector3 DeltaGyro;
};

class KalmanFilter
{
public:
	KalmanFilter();

	void Predict(float delta);

	void UpdateGPS();
	void UpdateMag();
	void UpdateBarom();
	void UpdateDeltaGyro();
	void UpdateDeltaAccel();

public:
	static const uint8_t NUM_STATES = 21;
	static const uint8_t NUM_SENSORS = 12 + (uint8_t)BAROMETER_ENABLED;

public:
	float SensorNoiseBarom;
	Vector3 SensorNoiseGPS;
	Vector3 SensorNoiseMag;
	Vector3 SensorNoiseDeltaGyro;
	Vector3 SensorNoiseDeltaAccel;

public:
	KalmanFilterState CurrentState{};
	KalmanFilterProcessInputs ProcessInputs{};
	KalmanFilterSensorReadings SensorReadings{};

private:
	void initSensorNoise();
	void initUpdateJacobians();

	void predictState(float delta);
	void predictJacobian(float delta);
	void predictCovariance(float delta);

	void updateState();
	void updateCovariance(Matrix& updateJacobian, Matrix& sensorNoise);

public:
	float m_IMU1Weight = 0.35f;
	Vector3 m_Accel{ 0.0f, 0.0f, 0.0f };
	Vector3 m_AngVel{ 0.0f, 0.0f, 0.0f };
	Vector3 m_MagFieldWorld{ 0.0f, sinf(3.1415f * 0.333f), cosf(3.1415f * 0.333f) };

public:
	Matrix m_ErrorEstimate{ NUM_STATES, 1 };
	Matrix m_ErrorCovariance = Matrix::Identity(NUM_STATES);
	Matrix m_SensorReadingsDif{ 3, 1 };

	Matrix m_ProcessNoise{ NUM_STATES, NUM_STATES }; // Matrix of size [NUM_STATES, NUM_STATES] representing the process noise covariance
	Matrix m_JacobianPredict = Matrix::Identity(NUM_STATES);

	Matrix m_JacobianUpdateGPS{ 3, NUM_STATES };
	Matrix m_JacobianUpdateMag{ 3, NUM_STATES };
	Matrix m_JacobianUpdateDeltaGyro{ 3, NUM_STATES };
	Matrix m_JacobianUpdateDeltaAccel{ 3, NUM_STATES };

private:
	Matrix m_CovarianceCorrectionBarom = Matrix::Identity(NUM_STATES);

public:
	Matrix m_MeasurementCovariance{ 3, 3 };
	Matrix m_KalmanGain{ 3, NUM_STATES }; // Value is then transposed to be [NUM_STATES, NUM_SENSORS] for the state update
	Matrix m_StateInnovation{ NUM_STATES, 1 };
	Matrix m_CovarianceCorrection{ NUM_STATES, NUM_STATES };

	Matrix m_ScratchMatrix1{ NUM_STATES, NUM_STATES };
	Matrix m_ScratchMatrix2{ NUM_STATES, 3 };
};


extern void OUTPUT_TEXT_ARES(const char* txt);
extern void OUTPUT_FLOAT_ARES(float num, uint8_t dp);
