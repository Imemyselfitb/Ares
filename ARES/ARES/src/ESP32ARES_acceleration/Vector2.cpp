#include "Vector2.h"

#include <cmath>

/****************************** CONST FUNCTIONS ******************************/
float Vector2::magSq() const
{
	return x * x + y * y;
}
float Vector2::mag() const
{
	return std::sqrt(x * x + y * y);
}
Vector2 Vector2::normalised() const
{
	float magnitude = mag();
	if (magnitude == 0.0)
		return Vector2{ 0.0, 0.0 };

	return Vector2{ x / magnitude, y / magnitude };
}

float Vector2::dot(const Vector2& other) const
{
	return x * other.x + y * other.y;
}


void Vector2::Print() const
{
	OUTPUT_TEXT_ARES("Vector2 ( x:");
	OUTPUT_FLOAT_ARES(x, 3);
	OUTPUT_TEXT_ARES(", y:");
	OUTPUT_FLOAT_ARES(y, 3);
	OUTPUT_TEXT_ARES(")");
}

/**************************** NON-CONST FUNCTIONS ****************************/
void Vector2::normalise()
{
	float magnitude = mag();
	if (magnitude == 0.0)
		return;

	x /= magnitude;
	y /= magnitude;
}
void Vector2::setMag(float newMag)
{
	float magnitude = mag();
	if (magnitude == 0.0)
		return;

	x *= newMag / magnitude;
	y *= newMag / magnitude;
}

void Vector2::clampAxis(float min, float max)
{
	if (x < min) x = min;
	else if (x > max) x = max;
	if (y < min) y = min;
	else if (y > max) y = max;
}

/****************************** CONST OPERATORS ******************************/
Vector2 Vector2::operator+(const Vector2& other) const
{
	return Vector2{ x + other.x, y + other.y };
}
Vector2 Vector2::operator-(const Vector2& other) const
{
	return Vector2{ x - other.x, y - other.y };
}
Vector2 Vector2::operator*(float scalar) const
{
	return Vector2{ x * scalar, y * scalar };
}
Vector2 Vector2::operator/(float scalar) const
{
	return Vector2{ x / scalar, y / scalar };
}

/**************************** NON-CONST OPERATORS ****************************/
void Vector2::operator+=(const Vector2& other)
{
	x += other.x;
	y += other.y;
}
void Vector2::operator-=(const Vector2& other)
{
	x -= other.x;
	y -= other.y;
}
void Vector2::operator*=(float scalar)
{
	x *= scalar;
	y *= scalar;
}
void Vector2::operator/=(float scalar)
{
	x /= scalar;
	y /= scalar;
}


std::ostream& operator<<(std::ostream& os, const Vector2& vec)
{
	os << "Vector2( x:" << vec.x << ", y:" << vec.y << " )";
	return os;
}
