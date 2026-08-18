#include "Vector3.h"

#include <cmath>

/****************************** CONST FUNCTIONS ******************************/
float Vector3::magSq() const
{
	return x * x + y * y + z * z;
}
float Vector3::mag() const
{
	return std::sqrt(x * x + y * y + z * z);
}
Vector3 Vector3::normalised() const
{
	float magnitude = mag();
	if (magnitude == 0.0f)
		return Vector3{ 0.0f, 0.0f, 0.0f };

	return Vector3{ x / magnitude, y / magnitude, z / magnitude };
}

float Vector3::dot(const Vector3& other) const
{
	return x * other.x + y * other.y + z * other.z;
}
Vector3 Vector3::cross(const Vector3& other) const
{
	return Vector3{
		y*other.z - z*other.y,
		z*other.x - x*other.z,
		x*other.y - y*other.x
	};
}


void Vector3::Print() const
{
	OUTPUT_TEXT_ARES("Vector3 ( x:");
	OUTPUT_FLOAT_ARES(x, 3);
	OUTPUT_TEXT_ARES(", y:");
	OUTPUT_FLOAT_ARES(y, 3);
	OUTPUT_TEXT_ARES(", z:");
	OUTPUT_FLOAT_ARES(z, 3);
	OUTPUT_TEXT_ARES(")");
}

void Vector3::PrintRaw() const
{
	OUTPUT_FLOAT_ARES(x, 3);
	OUTPUT_TEXT_ARES(", ");
	OUTPUT_FLOAT_ARES(y, 3);
	OUTPUT_TEXT_ARES(", ");
	OUTPUT_FLOAT_ARES(z, 3);
}

/**************************** NON-CONST FUNCTIONS ****************************/
void Vector3::normalise()
{
	float magnitude = mag();
	if (magnitude == 0.0)
		return;

	x /= magnitude;
	y /= magnitude;
	z /= magnitude;
}
void Vector3::setMag(float newMag)
{
	float magnitude = mag();
	if (magnitude == 0.0)
		return;

	x *= newMag / magnitude;
	y *= newMag / magnitude;
	z *= newMag / magnitude;
}

void Vector3::clampAxis(float min, float max)
{
	if (x < min) x = min;
	else if (x > max) x = max;
	if (y < min) y = min;
	else if (y > max) y = max;
	if (z < min) z = min;
	else if (z > max) z = max;
}

/****************************** CONST OPERATORS ******************************/
Vector3 Vector3::operator+(const Vector3& other) const
{
	return Vector3{ x + other.x, y + other.y, z + other.z };
}
Vector3 Vector3::operator-(const Vector3& other) const
{
	return Vector3{ x - other.x, y - other.y, z - other.z };
}
Vector3 Vector3::operator*(float scalar) const
{
	return Vector3{ x * scalar, y * scalar, z * scalar };
}
Vector3 Vector3::operator/(float scalar) const
{
	return Vector3{ x / scalar, y / scalar, z / scalar };
}

/**************************** NON-CONST OPERATORS ****************************/
void Vector3::operator+=(const Vector3& other)
{
	x += other.x;
	y += other.y;
	z += other.z;
}
void Vector3::operator-=(const Vector3& other)
{
	x -= other.x;
	y -= other.y;
	z -= other.z;
}
void Vector3::operator*=(float scalar)
{
	x *= scalar;
	y *= scalar;
	z *= scalar;
}
void Vector3::operator/=(float scalar)
{
	x /= scalar;
	y /= scalar;
	z /= scalar;
}
