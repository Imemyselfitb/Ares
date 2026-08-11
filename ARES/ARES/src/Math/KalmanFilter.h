#pragma once

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
	Vector3 GPS;
	Vector3 Mag;
	Vector3 DiffIMUAccel;
	Vector3 DiffIMUGyro;
#if BAROMETER_ENABLED
	float Barom;
#endif
};

class KalmanFilter
{
public:
	KalmanFilter();

	void Predict(float delta);
	void Update();

public:
	static const uint8_t NUM_STATES = 21;
	static const uint8_t NUM_PROCESS_INPUTS = 12;

#if BAROMETER_ENABLED
	static const uint8_t NUM_SENSORS = 13;
#else
	static const uint8_t NUM_SENSORS = 12;
#endif

public:
	KalmanFilterState CurrentState{};
	KalmanFilterProcessInputs ProcessInputs{};
	KalmanFilterSensorReadings SensorReadings{};

private:
	void predictState(float delta);
	void predictJacobian(float delta);
	void predictCovariance(float delta);

	void updateState();
	void updateSensorDifferences();
	void updateJacobian();
	void updateCovariance();

public:
	float m_IMU1Weight = 0.5f;
	Vector3 m_Accel{ 0.0f, 0.0f, 0.0f };
	Vector3 m_AngVel{ 0.0f, 0.0f, 0.0f };

	float m_TimeSinceUpdateGPS = 0.0f;
#if BAROMETER_ENABLED
	float m_TimeSinceUpdateBarometer = 0.0f;
#endif

	Vector3 m_MagFieldWorld{ 0.0f, sinf(3.1415f * 0.333f), cosf(3.1415f * 0.333f) };

private:
	Matrix m_ErrorEstimate{ NUM_STATES, 1 };
	Matrix m_ErrorCovariance = Matrix::Identity(NUM_STATES);
	Matrix m_SensorReadingsDif{ NUM_SENSORS, 1 };

	Matrix m_ProcessNoise; // Matrix of size [NUM_STATES, NUM_STATES] representing the process noise covariance
	Matrix m_SensorNoise; // Matrix of size [NUM_SENSORS, NUM_SENSORS] representing the sensor noise covariance

	Matrix m_JacobianPredict = Matrix::Identity(NUM_STATES);
	Matrix m_JacobianUpdate{ NUM_SENSORS, NUM_STATES };

private:
	Matrix m_MeasurementCovariance{ NUM_SENSORS, NUM_SENSORS };
	Matrix m_KalmanGain{ NUM_SENSORS, NUM_STATES }; // Value is then transposed to be [NUM_STATES, NUM_SENSORS] for the state update
	Matrix m_StateInnovation{ NUM_STATES, 1 };
	Matrix m_CovarianceCorrection{ NUM_STATES, NUM_STATES };

	Matrix m_ScratchMatrix1{ NUM_STATES, NUM_STATES };
	Matrix m_ScratchMatrix2{ NUM_STATES, NUM_SENSORS };
};
