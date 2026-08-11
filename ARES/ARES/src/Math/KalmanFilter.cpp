#include "KalmanFilter.h"

const float ProcessNoise[KalmanFilter::NUM_STATES] = {
	0.2f, 0.2f, 0.2f, // Position
	0.2f, 0.2f, 0.2f, // Velocity
	0.2f, 0.2f, 0.2f, // Orientation
	0.2f, 0.2f, 0.2f, // Bias Mean Accel
	0.2f, 0.2f, 0.2f, // Bias Mean Gyro
	0.2f, 0.2f, 0.2f, // Bias Delta Accel
	0.2f, 0.2f, 0.2f, // Bias Delta Gyro
};

const float SensorNoise[KalmanFilter::NUM_SENSORS] = {
	1.0f, 1.0f, 1.0f, // GPS Position
	0.01f, 0.01f, 0.01f, // Magnetometer
	0.5f, 0.5f, 0.5f, // Accel IMU Difference
	0.5f, 0.5f, 0.5f, // Gyro IMU Difference

#if BAROMETER_ENABLED
	10.0f // Barometer
#endif
};

KalmanFilter::KalmanFilter()
	: m_ProcessNoise(Matrix::Diagonal(NUM_STATES, ProcessNoise)),
	m_SensorNoise(Matrix::Diagonal(NUM_SENSORS, SensorNoise))
{
	// GPS Position
	m_JacobianUpdate(0, 0) = 1.0f;
	m_JacobianUpdate(1, 1) = 1.0f;
	m_JacobianUpdate(2, 2) = 1.0f;
	// Accel IMU Difference
	m_JacobianUpdate(6, 15) = -1.0f;
	m_JacobianUpdate(7, 16) = -1.0f;
	m_JacobianUpdate(8, 17) = -1.0f;
	// Gyro IMU Difference
	m_JacobianUpdate(9, 18) = -1.0f;
	m_JacobianUpdate(10, 19) = -1.0f;
	m_JacobianUpdate(11, 20) = -1.0f;

#if BAROMETER_ENABLED
	m_JacobianUpdate(12, 1) = 1.0f;
#endif
}

void KalmanFilter::Predict(float delta)
{
	predictState(delta);
	predictJacobian(delta);
	predictCovariance(delta);
}

void KalmanFilter::Update()
{
	updateSensorDifferences();
	updateJacobian();
	updateCovariance();
	updateState();
}

void KalmanFilter::predictState(float delta)
{
	// Subtract accel biases and orientate into body-frame
	Vector3 accel1 = ProcessInputs.Accel1 - (CurrentState.BiasMeanAccel + CurrentState.BiasDeltaAccel * 0.5f);
	Vector3 accel2 = ProcessInputs.Accel2 - (CurrentState.BiasMeanAccel - CurrentState.BiasDeltaAccel * 0.5f);
	Vector3 accelBody = accel1 * m_IMU1Weight + accel2 * (1.0f - m_IMU1Weight);
	m_Accel = CurrentState.Orientation.rotateVector(accelBody);

	// Subtract gyro biases (already in body-frame)
	Vector3 gyro1 = ProcessInputs.Gyro1 - (CurrentState.BiasMeanGyro + CurrentState.BiasDeltaGyro * 0.5f);
	Vector3 gyro2 = ProcessInputs.Gyro2 - (CurrentState.BiasMeanGyro - CurrentState.BiasDeltaGyro * 0.5f);
	m_AngVel = gyro1 * m_IMU1Weight + gyro2 * (1.0f - m_IMU1Weight);

	// Update current state
	CurrentState.Position += CurrentState.Velocity * delta + m_Accel * (0.5f * delta * delta);
	CurrentState.Velocity += m_Accel * delta;

	float speed = m_AngVel.mag() * delta;
	if (speed > 0.000001f)
	{
		Vector3 axis = m_AngVel.normalised();
		Quaternion deltaOrientation{ axis * std::sin(speed * 0.5f), std::cos(speed * 0.5f) };
		CurrentState.Orientation = (CurrentState.Orientation * deltaOrientation).normalised();
	}
}

void KalmanFilter::predictJacobian(float delta)
{
	// Perror = Verror
	m_JacobianPredict(0, 3) = delta;
	m_JacobianPredict(1, 4) = delta;
	m_JacobianPredict(2, 5) = delta;

	float rotationMatrix[9];
	CurrentState.Orientation.toRotationMatrix(rotationMatrix);

	float deltaIMUScale = 1.0f - 2.0f * m_IMU1Weight;
	for (uint8_t i = 0; i < 3; ++i)
	{
		// Verror = -R(q)*S(a)*Qerror - R(q)*BAerror + R(q)*(1-2w)*BADerror
		m_JacobianPredict(3 + i, 6) = delta * (rotationMatrix[2 + i * 3] * m_Accel.y - rotationMatrix[1 + i * 3] * m_Accel.z);
		m_JacobianPredict(3 + i, 7) = delta * (rotationMatrix[0 + i * 3] * m_Accel.z - rotationMatrix[2 + i * 3] * m_Accel.x);
		m_JacobianPredict(3 + i, 8) = delta * (rotationMatrix[1 + i * 3] * m_Accel.x - rotationMatrix[0 + i * 3] * m_Accel.y);
		m_JacobianPredict(3 + i, 9) = delta * -rotationMatrix[0 + i * 3];
		m_JacobianPredict(3 + i, 10) = delta * -rotationMatrix[1 + i * 3];
		m_JacobianPredict(3 + i, 11) = delta * -rotationMatrix[2 + i * 3];
		m_JacobianPredict(3 + i, 15) = delta * rotationMatrix[0 + i * 3] * deltaIMUScale;
		m_JacobianPredict(3 + i, 16) = delta * rotationMatrix[1 + i * 3] * deltaIMUScale;
		m_JacobianPredict(3 + i, 17) = delta * rotationMatrix[2 + i * 3] * deltaIMUScale;
	}

	// Qerror = -S(w)*Qerror - BGerror - (1-2w)*BGDerror
	m_JacobianPredict(6, 7) = delta * m_AngVel.z;
	m_JacobianPredict(6, 8) = delta * -m_AngVel.y;
	m_JacobianPredict(6, 12) = -delta;
	m_JacobianPredict(6, 18) = delta * deltaIMUScale;

	m_JacobianPredict(7, 6) = delta * -m_AngVel.z;
	m_JacobianPredict(7, 8) = delta * m_AngVel.x;
	m_JacobianPredict(7, 13) = -delta;
	m_JacobianPredict(7, 19) = delta * deltaIMUScale;

	m_JacobianPredict(8, 6) = delta * m_AngVel.y;
	m_JacobianPredict(8, 7) = delta * -m_AngVel.x;
	m_JacobianPredict(8, 14) = -delta;
	m_JacobianPredict(8, 20) = delta * deltaIMUScale;
}

void KalmanFilter::predictCovariance(float delta)
{
	// stateCovariance = jacobian.dot(stateCovariance).dot(jacobian.transposed()).add(processNoise)
	m_ScratchMatrix1.AssignDotProduct(m_JacobianPredict, m_ErrorCovariance);
	m_JacobianPredict.Transpose();
	m_ErrorCovariance.AssignDotProduct(m_ScratchMatrix1, m_JacobianPredict);
	m_ErrorCovariance += m_ProcessNoise;
}

void KalmanFilter::updateState()
{
	CurrentState.Position.x += m_StateInnovation(0, 0);
	CurrentState.Position.y += m_StateInnovation(1, 0);
	CurrentState.Position.z += m_StateInnovation(2, 0);

	CurrentState.Velocity.x += m_StateInnovation(3, 0);
	CurrentState.Velocity.y += m_StateInnovation(4, 0);
	CurrentState.Velocity.z += m_StateInnovation(5, 0);

	Quaternion deltaOrientation{
		Vector3{ m_StateInnovation(6, 0), m_StateInnovation(7, 0), m_StateInnovation(8, 0) } * 0.5f,
		1.0f
	};
	CurrentState.Orientation = (CurrentState.Orientation * deltaOrientation).normalised();

	CurrentState.BiasMeanAccel.x += m_StateInnovation(9, 0);
	CurrentState.BiasMeanAccel.y += m_StateInnovation(10, 0);
	CurrentState.BiasMeanAccel.z += m_StateInnovation(11, 0);
	CurrentState.BiasMeanGyro.x += m_StateInnovation(12, 0);
	CurrentState.BiasMeanGyro.y += m_StateInnovation(13, 0);
	CurrentState.BiasMeanGyro.z += m_StateInnovation(14, 0);

	CurrentState.BiasDeltaAccel.x += m_StateInnovation(15, 0);
	CurrentState.BiasDeltaAccel.y += m_StateInnovation(16, 0);
	CurrentState.BiasDeltaAccel.z += m_StateInnovation(17, 0);
	CurrentState.BiasDeltaGyro.x += m_StateInnovation(18, 0);
	CurrentState.BiasDeltaGyro.y += m_StateInnovation(19, 0);
	CurrentState.BiasDeltaGyro.z += m_StateInnovation(20, 0);
}

void KalmanFilter::updateSensorDifferences()
{
	Vector3 predictedGPS = CurrentState.Position + CurrentState.Velocity * m_TimeSinceUpdateGPS;
	m_SensorReadingsDif(0, 0) = SensorReadings.GPS.x - predictedGPS.x;
	m_SensorReadingsDif(1, 0) = SensorReadings.GPS.y - predictedGPS.y;
	m_SensorReadingsDif(2, 0) = SensorReadings.GPS.z - predictedGPS.z;

	Vector3 predictedMag = CurrentState.Orientation.conjugate().rotateVector(m_MagFieldWorld);
	m_SensorReadingsDif(3, 0) = SensorReadings.Mag.x - predictedMag.x;
	m_SensorReadingsDif(4, 0) = SensorReadings.Mag.y - predictedMag.y;
	m_SensorReadingsDif(5, 0) = SensorReadings.Mag.z - predictedMag.z;

	Vector3 predictedDiffIMUAccel = ProcessInputs.Accel1 - ProcessInputs.Accel2;
	m_SensorReadingsDif(6, 0) = SensorReadings.DiffIMUAccel.x - predictedDiffIMUAccel.x;
	m_SensorReadingsDif(7, 0) = SensorReadings.DiffIMUAccel.y - predictedDiffIMUAccel.y;
	m_SensorReadingsDif(8, 0) = SensorReadings.DiffIMUAccel.z - predictedDiffIMUAccel.z;

	Vector3 predictedDiffIMUGyro = ProcessInputs.Gyro1 - ProcessInputs.Gyro2;
	m_SensorReadingsDif(9, 0) = SensorReadings.DiffIMUGyro.x - predictedDiffIMUGyro.x;
	m_SensorReadingsDif(10, 0) = SensorReadings.DiffIMUGyro.y - predictedDiffIMUGyro.y;
	m_SensorReadingsDif(11, 0) = SensorReadings.DiffIMUGyro.z - predictedDiffIMUGyro.z;

#if BAROMETER_ENABLED
	float predictedBarometer = CurrentState.Position.z + CurrentState.Velocity.z * m_TimeSinceUpdateGPS;
	m_SensorReadingsDif(12, 0) = SensorReadings.Barom - predictedBarometer;
#endif
}

void KalmanFilter::updateJacobian()
{
	// GPS Position
	m_JacobianUpdate(0, 3) = m_TimeSinceUpdateGPS;
	m_JacobianUpdate(1, 4) = m_TimeSinceUpdateGPS;
	m_JacobianUpdate(2, 5) = m_TimeSinceUpdateGPS;
	// Magnetometer
	Vector3 north = CurrentState.Orientation.conjugate().rotateVector(m_MagFieldWorld);
	m_JacobianUpdate(3, 7) = -north.z;
	m_JacobianUpdate(3, 8) = north.y;
	m_JacobianUpdate(4, 6) = north.z;
	m_JacobianUpdate(4, 8) = -north.x;
	m_JacobianUpdate(5, 6) = -north.y;
	m_JacobianUpdate(5, 7) = north.x;

#if BAROMETER_ENABLED
	m_JacobianUpdate(12, 4) = m_TimeSinceUpdateGPS;
#endif
}

void KalmanFilter::updateCovariance()
{
	// measurementCovariance = jacobian.dot(stateCovariance.dot(jacobianTransp)).add(sensorNoise)
	m_KalmanGain.AssignDotProduct(m_JacobianUpdate, m_ErrorCovariance); // [not currently the kalman gain]
	m_JacobianUpdate.Transpose(); // jacobian -> jacobianTransp
	m_MeasurementCovariance.AssignDotProduct(m_KalmanGain, m_JacobianUpdate);
	m_JacobianUpdate.Transpose(); // jacobianTransp -> jacobian
	m_MeasurementCovariance += m_SensorNoise;

	// kalmanGain = stateCovariance.dot(jacobianTransp).dot(measurementCovariance.inv())
	// [NOW USES CHOLESKY SOLVING!!!!]: kalmanGain = jacobian.dot(stateCovariance).solve(measurementCovariance.cholesky()).transposed()
	m_MeasurementCovariance.CholeskyDecompose();
	m_KalmanGain.SolveCholesky(m_MeasurementCovariance);
	m_KalmanGain.Transpose();

	// stateInnovation = kalmanGain.dot(dif)
	m_StateInnovation.AssignDotProduct(m_KalmanGain, m_SensorReadingsDif);

	// correction = identityState.add(kalmanGain.dot(jacobian).scale(-1))
	m_CovarianceCorrection.AssignDotProduct(m_KalmanGain, m_JacobianUpdate);
	m_CovarianceCorrection *= -1.0f;
	for (uint8_t i = 0; i < m_CovarianceCorrection.rows; ++i)
		m_CovarianceCorrection.data[i * m_CovarianceCorrection.cols + i] += 1.0f;
	
	// stateCovariance = correction.dot(stateCovariance).dot(correction.transposed())
	m_ScratchMatrix1.AssignDotProduct(m_CovarianceCorrection, m_ErrorCovariance);
	m_CovarianceCorrection.Transpose();
	m_ErrorCovariance.AssignDotProduct(m_ScratchMatrix1, m_CovarianceCorrection);

	// stateCovariance = stateCovariance.add(kalmanGain.dot(sensorNoise).dot(kalmanGain.transposed()))
	m_ScratchMatrix2.AssignDotProduct(m_KalmanGain, m_SensorNoise);
	m_KalmanGain.Transpose();
	m_ScratchMatrix1.AssignDotProduct(m_ScratchMatrix2, m_KalmanGain);
	m_ErrorCovariance += m_ScratchMatrix1;
}
