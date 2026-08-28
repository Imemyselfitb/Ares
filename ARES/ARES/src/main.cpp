#include <stdio.h>
#include <stdint.h>

#include "ESP32ARES_acceleration/KalmanFilter.h"
extern void OUTPUT_TEXT_ARES(const char* txt)
{
	printf(txt);
}
extern void OUTPUT_FLOAT_ARES(float num, uint8_t dp)
{
	printf("%.*f", dp, num);
}

int main()
{
	KalmanFilter KF;

	Vector3 bias1{ 0.1f, 0.2f, -0.3f };
	Vector3 bias2{ -0.1f, 0.09f, -0.05f };

	Vector3 TRUE_UP_1{ 0.0f, 9.802f, -0.2942f };
	Vector3 TRUE_UP_2{ 0.4901f , 9.794f, 0.0f };
	
	Vector3 up1 = bias1 + TRUE_UP_1;
	Vector3 down1 = bias1 - TRUE_UP_1;
	Vector3 up2 = bias2 + TRUE_UP_2;
	Vector3 down2 = bias2 - TRUE_UP_2;

	KF.CalibrateIMURotationalOffset(up1, down1, up2, down2);

	Vector3 AVG_CORRECT_UP{ 0.0f, 8.609f, 4.703f };
	KF.CalibrateInitialState(AVG_CORRECT_UP, AVG_CORRECT_UP);

	Vector3 up = KF.CurrentState.Orientation.rotateVector(AVG_CORRECT_UP);
	printf("Orientation: ");
	up.Print();
	printf("\n");
}
