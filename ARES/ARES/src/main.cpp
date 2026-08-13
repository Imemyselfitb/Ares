#include "Math/KalmanFilter.h"

#include "KalmanTestData.h"

#include <chrono>

int main(int argc, char* argv[])
{
#if 0

#define MAT_DATA_A +3.6f
#define MAT_DATA_B +0.9f
#define MAT_DATA_C +1.5f
#define MAT_DATA_D +4.5f
#define MAT_DATA_E +3.3f
#define MAT_DATA_F +4.9f

#define MAT_DATA { \
		MAT_DATA_A, MAT_DATA_B, MAT_DATA_C, \
		MAT_DATA_B, MAT_DATA_D, MAT_DATA_E, \
		MAT_DATA_C, MAT_DATA_E, MAT_DATA_F \
	}

	float matrixData[9] = MAT_DATA;
	Matrix A{ 3, 3, matrixData };
	A.CholeskyDecompose();

	float BmatrixData[6] = {
		0.8f, 0.1f, 
		0.1f, 3.2f,
		0.9f, 3.4f,
	};

	//Matrix B{ 1, 3, BmatrixData };
	Matrix B{ 3, 2, BmatrixData };
	
	B.SolveCholesky(A);
	
	float matrixDataA[9] = MAT_DATA;
	Matrix Acopy{ 3, 3, matrixDataA };

	Matrix R{ 3, 2 };
	R.AssignDotProduct(Acopy, B);

	std::cout << std::setprecision(2) << "Matrix R:" << R << std::endl;

	B.Transpose();
	std::cout << std::setprecision(2) << "'Kalman Gain' K:" << B << std::endl;


#else


	KalmanFilter kf;

	auto start = std::chrono::high_resolution_clock::now();
	const uint32_t iterations = 100;

	float prevTime = 0.0f;
	for (uint32_t i = 0; i < iterations; i++)
	{
		float curTime = KalmanData[i][0];
		float deltaTime = curTime - prevTime;
		prevTime = curTime;

		//----------------------IMU------------------------//
		kf.ProcessInputs.Accel1 = Vector3{ KalmanData[i][1], KalmanData[i][2], KalmanData[i][3] };
		kf.ProcessInputs.Gyro1 = Vector3{ KalmanData[i][4], KalmanData[i][5], KalmanData[i][6] };
		kf.ProcessInputs.Accel2 = Vector3{ KalmanData[i][7], KalmanData[i][8], KalmanData[i][9] };
		kf.ProcessInputs.Gyro2 = Vector3{ KalmanData[i][10], KalmanData[i][11], KalmanData[i][12] };

		kf.SensorReadings.DeltaAccel = kf.ProcessInputs.Accel1 - kf.ProcessInputs.Accel2;
		kf.SensorReadings.DeltaGyro = kf.ProcessInputs.Gyro1 - kf.ProcessInputs.Gyro2;

		kf.Predict(deltaTime);
		kf.UpdateDeltaAccel();
		kf.UpdateDeltaGyro();

		//----------------------Sensors------------------------//
		kf.SensorReadings.GPS = Vector3{ KalmanData[i][14], KalmanData[i][15], KalmanData[i][16] };
		kf.UpdateGPS();

		kf.SensorReadings.Mag = Vector3{ KalmanData[i][17], KalmanData[i][18], KalmanData[i][19] };
		kf.UpdateMag();

#if BAROMETER_ENABLED
		kf.SensorReadings.Barom = KalmanData[i][13];
		kf.UpdateBarom();
#endif

		//std::cout << "Position: " << kf.CurrentState.Position << std::endl;
		//std::cout << "Velocity: " << kf.CurrentState.Velocity << std::endl;
		//std::cout << "Nose Orientation: " << kf.CurrentState.Orientation.rotateVector(Vector3(0.0, 1.0, 0.0)) << std::endl;
	}

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	std::cout << "Duration: " << duration << "\xE6s/" << iterations << "x" << std::endl;
	std::cout << "Average Duration: " << duration / iterations << "\xE6s" << std::endl;


#endif
}
