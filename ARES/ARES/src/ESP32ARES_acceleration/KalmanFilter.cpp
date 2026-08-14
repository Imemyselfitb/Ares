#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
	initSensorNoise();
	initUpdateJacobians();
}

// Values are modifiable
void KalmanFilter::initSensorNoise()
{
	SensorNoiseBarom = 1.0f;
	SensorNoiseGPS = Vector3{ 1.0f, 1.0f, 1.0f };
	SensorNoiseMag = Vector3{ 0.01f, 0.01f, 0.01f };
	//BIASES:
	SensorNoiseDeltaGyro = Vector3{ 0.5f, 0.5f, 0.5f };
	SensorNoiseDeltaAccel = Vector3{ 0.5f, 0.5f, 0.5f };

	const float processNoise[NUM_STATES] = {
		0.2f, 0.2f, 0.2f, // Position
		0.2f, 0.2f, 0.2f, // Velocity
		0.2f, 0.2f, 0.2f, // Orientation
		0.2f, 0.2f, 0.2f, // Bias Mean Accel
		0.2f, 0.2f, 0.2f, // Bias Mean Gyro
		0.2f, 0.2f, 0.2f, // Bias Delta Accel
		0.2f, 0.2f, 0.2f, // Bias Delta Gyro
	};

	for (uint8_t i = 0; i < NUM_STATES; i++)
		m_ProcessNoise.data[i * NUM_STATES + i] = processNoise[i];
}

void KalmanFilter::initUpdateJacobians()
{
	m_JacobianUpdateGPS(0, 0) = 1.0f;
	m_JacobianUpdateGPS(1, 1) = 1.0f;
	m_JacobianUpdateGPS(2, 2) = 1.0f;

	m_JacobianUpdateDeltaAccel(0, 15) = -1.0f;
	m_JacobianUpdateDeltaAccel(1, 16) = -1.0f;
	m_JacobianUpdateDeltaAccel(2, 17) = -1.0f;

	m_JacobianUpdateDeltaGyro(0, 18) = -1.0f;
	m_JacobianUpdateDeltaGyro(1, 19) = -1.0f;
	m_JacobianUpdateDeltaGyro(2, 20) = -1.0f;
}

void KalmanFilter::CalibrateIMURotationalOffset(const Vector3 &AverageAccelUpIMU1, const Vector3 &AverageAccelDownIMU1, const Vector3 &AverageAccelUpIMU2, const Vector3 &AverageAccelDownIMU2)
{
	Vector3 accBias1 = (AverageAccelUpIMU1 + AverageAccelDownIMU1) * 0.5f;
	Vector3 accBias2 = (AverageAccelUpIMU2 + AverageAccelDownIMU2) * 0.5f;
	CurrentState.BiasMeanAccel = (accBias1 + accBias2) * 0.5f;
	CurrentState.BiasDeltaAccel = accBias1 - accBias2;

	Vector3 upIMU1 = (AverageAccelUpIMU1 - AverageAccelDownIMU1) * 0.5f;
	Vector3 upIMU2 = (AverageAccelUpIMU2 - AverageAccelDownIMU2) * 0.5f;
	m_SensorAlignmentIMU1 = Quaternion{ upIMU1, Vector3{ 0.0f, upIMU1.mag(), 0.0f } };
	m_SensorAlignmentIMU2 = Quaternion{ upIMU2, Vector3{ 0.0f, upIMU2.mag(), 0.0f } };
}

void KalmanFilter::CorrectIMUReadings()
{
	ProcessInputs.Accel1 = m_SensorAlignmentIMU1.rotateVector(ProcessInputs.Accel1 - (CurrentState.BiasMeanAccel + CurrentState.BiasDeltaAccel * 0.5f));
	ProcessInputs.Gyro1 = m_SensorAlignmentIMU1.rotateVector(ProcessInputs.Gyro1 - (CurrentState.BiasMeanGyro + CurrentState.BiasDeltaGyro * 0.5f));
	ProcessInputs.Accel2 = m_SensorAlignmentIMU2.rotateVector(ProcessInputs.Accel2 - (CurrentState.BiasMeanAccel - CurrentState.BiasDeltaAccel * 0.5f));
	ProcessInputs.Gyro2 = m_SensorAlignmentIMU2.rotateVector(ProcessInputs.Gyro2 - (CurrentState.BiasMeanGyro - CurrentState.BiasDeltaGyro * 0.5f));
}

void KalmanFilter::CalibrateInitialState(const Vector3& AverageCorrectedAccelIMU1, const Vector3& AverageCorrectedAccelIMU2)
{
	// Assumes the rocket is stationary
	CurrentState.Position *= 0.0f;
	CurrentState.Velocity *= 0.0f;

	Vector3 accelBody = AverageCorrectedAccelIMU1 * m_IMU1Weight + AverageCorrectedAccelIMU2 * (1.0f - m_IMU1Weight);
	CurrentState.Orientation = Quaternion{ accelBody, Vector3{ 0.0f, accelBody.mag(), 0.0f } };
}

void KalmanFilter::Predict(float delta)
{
	predictState(delta);
	predictJacobian(delta);
	predictCovariance(delta);
}

void KalmanFilter::predictState(float delta)
{
	// NOTE: Accel and Gyro readings have been corrected by `CorrectIMUReadings()`

	// Weight acceleration biases and orientate into body-frame
	Vector3 accelBody = ProcessInputs.Accel1 * m_IMU1Weight + ProcessInputs.Accel2 * (1.0f - m_IMU1Weight);
	m_Accel = CurrentState.Orientation.rotateVector(accelBody) - Vector3(0.0f, 9.80665f, 0.0f);

	// Weight angular velocity (already in body-frame)
	m_AngVel = ProcessInputs.Gyro1 * m_IMU1Weight + ProcessInputs.Gyro2 * (1.0f - m_IMU1Weight);

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
	m_ErrorCovariance.AssignDotProduct(m_ScratchMatrix1, m_JacobianPredict.Transposed());
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

void KalmanFilter::UpdateBarom()
{
	float predictedBarom = CurrentState.Position.y;
	float diff = SensorReadings.Barom - predictedBarom;
	
	// Since only one reading is provided, the matrices can simplified and calculated manually
	float scale = 1.0f / (m_ErrorCovariance.data[NUM_STATES + 1] + SensorNoiseBarom);
	for (uint8_t i = 0; i < NUM_STATES; i++)
	{
		float kalman = m_ErrorCovariance.data[i * NUM_STATES + 1] * scale;
		m_StateInnovation.data[i] = kalman; // StateInnovation currently is kalman gain (until multiplied by diff)
		m_CovarianceCorrectionBarom.data[i * NUM_STATES + 1] = -kalman;
	}
	
	m_CovarianceCorrectionBarom.data[NUM_STATES + 1] += 1.0;

	// stateCovariance = correction.dot(stateCovariance).dot(correction.transposed())
	m_ScratchMatrix1.AssignDotProduct(m_CovarianceCorrectionBarom, m_ErrorCovariance);
	m_ErrorCovariance.AssignDotProduct(m_ScratchMatrix1, m_CovarianceCorrectionBarom.Transposed());

	// stateCovariance = stateCovariance.add(kalmanGain.dot(sensorNoise).dot(kalmanGain.transposed()))
	m_ScratchMatrix1.AssignDotProduct(m_StateInnovation, m_StateInnovation.Transposed());
	m_ScratchMatrix1 *= SensorNoiseBarom;
	m_ErrorCovariance += m_ScratchMatrix1;

	m_StateInnovation *= diff;
	updateState();
}

void KalmanFilter::UpdateGPS()
{
	Vector3 predictedGPS = CurrentState.Position;
	m_SensorReadingsDif(0, 0) = SensorReadings.GPS.x - predictedGPS.x;
	m_SensorReadingsDif(1, 0) = SensorReadings.GPS.y - predictedGPS.y;
	m_SensorReadingsDif(2, 0) = SensorReadings.GPS.z - predictedGPS.z;

	float sensorNoiseData[9] = {
		SensorNoiseGPS.x, 0.0, 0.0,
		0.0, SensorNoiseGPS.y, 0.0,
		0.0, 0.0, SensorNoiseGPS.z
	};
	Matrix sensorNoise{ 3, 3, sensorNoiseData };
	updateCovariance(m_JacobianUpdateGPS, sensorNoise);
	updateState();
}

void KalmanFilter::UpdateMag()
{
	Vector3 predictedMag = CurrentState.Orientation.conjugate().rotateVector(m_MagFieldWorld);
	m_SensorReadingsDif(0, 0) = SensorReadings.Mag.x - predictedMag.x;
	m_SensorReadingsDif(1, 0) = SensorReadings.Mag.y - predictedMag.y;
	m_SensorReadingsDif(2, 0) = SensorReadings.Mag.z - predictedMag.z;

	m_JacobianUpdateMag(0, 7) = -predictedMag.z;
	m_JacobianUpdateMag(0, 8) = predictedMag.y;
	m_JacobianUpdateMag(1, 6) = predictedMag.z;
	m_JacobianUpdateMag(1, 8) = -predictedMag.x;
	m_JacobianUpdateMag(2, 6) = -predictedMag.y;
	m_JacobianUpdateMag(2, 7) = predictedMag.x;

	float sensorNoiseData[9] = {
		SensorNoiseMag.x, 0.0, 0.0,
		0.0, SensorNoiseMag.y, 0.0,
		0.0, 0.0, SensorNoiseMag.z
	};
	Matrix sensorNoise{ 3, 3, sensorNoiseData };
	updateCovariance(m_JacobianUpdateMag, sensorNoise);
	updateState();
}

void KalmanFilter::UpdateDeltaGyro()
{
	Vector3 predictedDeltaGyro = CurrentState.BiasDeltaGyro * -1.0f;
	m_SensorReadingsDif(0, 0) = SensorReadings.DeltaGyro.x - predictedDeltaGyro.x;
	m_SensorReadingsDif(1, 0) = SensorReadings.DeltaGyro.y - predictedDeltaGyro.y;
	m_SensorReadingsDif(2, 0) = SensorReadings.DeltaGyro.z - predictedDeltaGyro.z;

	float sensorNoiseData[9] = {
		SensorNoiseDeltaGyro.x, 0.0, 0.0,
		0.0, SensorNoiseDeltaGyro.y, 0.0,
		0.0, 0.0, SensorNoiseDeltaGyro.z
	};
	Matrix sensorNoise{ 3, 3, sensorNoiseData };
	updateCovariance(m_JacobianUpdateDeltaGyro, sensorNoise);
	updateState();
}

void KalmanFilter::UpdateDeltaAccel()
{
	Vector3 predictedDeltaAccel = CurrentState.BiasDeltaAccel * -1.0f;
	m_SensorReadingsDif(0, 0) = SensorReadings.DeltaAccel.x - predictedDeltaAccel.x;
	m_SensorReadingsDif(1, 0) = SensorReadings.DeltaAccel.y - predictedDeltaAccel.y;
	m_SensorReadingsDif(2, 0) = SensorReadings.DeltaAccel.z - predictedDeltaAccel.z;

	float sensorNoiseData[9] = {
		SensorNoiseDeltaAccel.x, 0.0, 0.0,
		0.0, SensorNoiseDeltaAccel.y, 0.0,
		0.0, 0.0, SensorNoiseDeltaAccel.z
	};
	Matrix sensorNoise{ 3, 3, sensorNoiseData };
	updateCovariance(m_JacobianUpdateDeltaAccel, sensorNoise);
	updateState();
}

void KalmanFilter::updateCovariance(Matrix& updateJacobian, Matrix& sensorNoise)
{
	// measurementCovariance = jacobian.dot(stateCovariance).dot(jacobianTransp).add(sensorNoise)
	m_KalmanGain.AssignDotProduct(updateJacobian, m_ErrorCovariance); // [not currently the kalman gain]
	m_MeasurementCovariance.AssignDotProduct(m_KalmanGain, updateJacobian.Transposed());
	m_MeasurementCovariance += sensorNoise;

	// kalmanGain = stateCovariance.dot(jacobianTransp).dot(measurementCovariance.inv())
	// [NOW USES CHOLESKY SOLVING!!!!]: kalmanGain = jacobian.dot(stateCovariance).solve(measurementCovariance.cholesky()).transposed()
	bool success = m_MeasurementCovariance.CholeskyDecompose();
	m_KalmanGain.SolveCholesky(m_MeasurementCovariance);

	// OUTPUT_FLOAT_ARES(m_MeasurementCovariance(0,0), 3);

	m_KalmanGain.Transpose();

	// stateInnovation = kalmanGain.dot(dif)
	m_StateInnovation.AssignDotProduct(m_KalmanGain, m_SensorReadingsDif);

	// correction = identityState.add(kalmanGain.dot(jacobian).scale(-1))
	m_CovarianceCorrection.AssignDotProduct(m_KalmanGain, updateJacobian);
	m_CovarianceCorrection *= -1.0f;
	for (uint8_t i = 0; i < m_CovarianceCorrection.rows; ++i)
		m_CovarianceCorrection.data[i * m_CovarianceCorrection.cols + i] += 1.0f;
	
	// stateCovariance = correction.dot(stateCovariance).dot(correction.transposed())
	m_ScratchMatrix1.AssignDotProduct(m_CovarianceCorrection, m_ErrorCovariance);
	m_ErrorCovariance.AssignDotProduct(m_ScratchMatrix1, m_CovarianceCorrection.Transposed());

	// stateCovariance = stateCovariance.add(kalmanGain.dot(sensorNoise).dot(kalmanGain.transposed()))
	m_ScratchMatrix2.AssignDotProduct(m_KalmanGain, sensorNoise);
	m_KalmanGain.Transpose();
	m_ScratchMatrix1.AssignDotProduct(m_ScratchMatrix2, m_KalmanGain);
	m_ErrorCovariance += m_ScratchMatrix1;
}
