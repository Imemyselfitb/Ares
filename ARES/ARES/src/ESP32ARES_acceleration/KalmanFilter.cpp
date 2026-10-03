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
	SensorNoiseGPS = Vector3{ 0.4f, 1.6f, 0.4f };
	SensorNoiseMag = Vector3{ 3.61f, 20.25f, 3.61f };
	//BIASES:
	SensorNoiseDeltaGyro = Vector3{ 0.0006410f, 0.0006410f, 0.0006410f };
	SensorNoiseDeltaAccel = Vector3{ 0.03461f, 0.03461f, 0.03461f };

	// noise(accel1 - accel2)

	const float processNoise[NUM_STATES] = {
		0.2f, 0.2f, 0.2f, // Position
		0.2f, 0.2f, 0.2f, // Velocity
		0.2f, 0.2f, 0.2f, // Orientation
	};

	for (uint8_t i = 0; i < NUM_STATES; i++)
		m_ProcessNoise.data[i * NUM_STATES + i] = processNoise[i];
}

void KalmanFilter::initUpdateJacobians()
{
	m_JacobianUpdateGPS(0, 0) = 1.0f;
	m_JacobianUpdateGPS(1, 1) = 1.0f;
	m_JacobianUpdateGPS(2, 2) = 1.0f;
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

	// OUTPUT_TEXT_ARES("Accel Body: ");
	// accelBody.Print();
	// OUTPUT_TEXT_ARES("; Accel: ");
	// m_Accel.Print();
	// OUTPUT_TEXT_ARES("\n");

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

	Vector3 accelBody = ProcessInputs.Accel1 * m_IMU1Weight + ProcessInputs.Accel2 * (1.0f - m_IMU1Weight);

	for (uint8_t i = 0; i < 3; i++)
	{
		// Verror = -R(q)*S(a)*Qerror
		m_JacobianPredict(3 + i, 6) = delta * (rotationMatrix[i*3 + 2] * accelBody.y - rotationMatrix[i*3 + 1] * accelBody.z);
		m_JacobianPredict(3 + i, 7) = delta * (rotationMatrix[i*3 + 0] * accelBody.z - rotationMatrix[i*3 + 2] * accelBody.x);
		m_JacobianPredict(3 + i, 8) = delta * (rotationMatrix[i*3 + 1] * accelBody.x - rotationMatrix[i*3 + 0] * accelBody.y);
	}

	// Qerror = -S(w)*Qerror
	m_JacobianPredict(6, 7) = delta * m_AngVel.z;
	m_JacobianPredict(6, 8) = delta * -m_AngVel.y;
	m_JacobianPredict(7, 6) = delta * -m_AngVel.z;
	m_JacobianPredict(7, 8) = delta * m_AngVel.x;
	m_JacobianPredict(8, 6) = delta * m_AngVel.y;
	m_JacobianPredict(8, 7) = delta * -m_AngVel.x;
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
		m_StateInnovation.data[i] = kalman; // StateInnovation currently stores KalmanGain (until multiplied by diff) - [[Optimization]]
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

	m_StateInnovation *= diff; // StateInnovation no longer stores KalmanGain
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
