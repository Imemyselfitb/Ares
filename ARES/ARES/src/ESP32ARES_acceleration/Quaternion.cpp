#include "Quaternion.h"
#include <cmath>

Quaternion::Quaternion(const Vector3& from, const Vector3& to)
{
	w = 1.0f + from.dot(to);

	Vector3 cross = from.cross(to);
	x = cross.x;
	y = cross.y;
	z = cross.z;

	if (std::abs(w) < 0.000001)
	{
		w = 1.0;
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
}

/****************************** CONST FUNCTIONS ******************************/
float Quaternion::magSq() const
{
	return w * w + x * x + y * y + z * z;
}
float Quaternion::mag() const
{
	return std::sqrt(w * w + x * x + y * y + z * z);
}
Quaternion Quaternion::normalised() const
{
	float magnitude = mag();
	if (magnitude == 0.0)
		return Quaternion{ 1.0, 0.0, 0.0, 0.0 };

	return Quaternion{ w / magnitude, x / magnitude, y / magnitude, z / magnitude };
}
Quaternion Quaternion::conjugate() const
{
	return Quaternion{ w, -x, -y, -z };
}
Quaternion Quaternion::inverse() const
{
	float magnitudeSq = magSq();
	if (magnitudeSq == 0.0)
		return Quaternion{ 1.0, 0.0, 0.0, 0.0 };

	return Quaternion{ w / magnitudeSq, -x / magnitudeSq, -y / magnitudeSq, -z / magnitudeSq };
}

Vector3 Quaternion::getVector() const
{
	return Vector3{ x, y, z };
}

Vector3 Quaternion::rotateVector(const Vector3& point) const
{
	Quaternion projectedPoint = (*this) * Quaternion{ point } * conjugate();
	return projectedPoint.getVector();
}

void Quaternion::toRotationMatrix(float* matrix) const
{
	matrix[0] = 1.0f - 2.0f * (y * y + z * z);
	matrix[1] = 2.0f * (x * y - z * w);
	matrix[2] = 2.0f * (x * z + y * w);

	matrix[3] = 2.0f * (x * y + z * w);
	matrix[4] = 1.0f - 2.0f * (x * x + z * z);
	matrix[5] = 2.0f * (y * z - x * w);

	matrix[6] = 2.0f * (x * z - y * w);
	matrix[7] = 2.0f * (y * z + x * w);
	matrix[8] = 1.0f - 2.0f * (x * x + y * y);
}


void Quaternion::Print() const
{
	OUTPUT_TEXT_ARES("Quaternion ( w:");
	OUTPUT_FLOAT_ARES(w, 3);
	OUTPUT_TEXT_ARES(", x:");
	OUTPUT_FLOAT_ARES(x, 3);
	OUTPUT_TEXT_ARES(", y:");
	OUTPUT_FLOAT_ARES(y, 3);
	OUTPUT_TEXT_ARES(", z:");
	OUTPUT_FLOAT_ARES(z, 3);
	OUTPUT_TEXT_ARES(")");
}

/****************************** CONST OPERATORS ******************************/
Quaternion Quaternion::operator+(const Quaternion& other) const
{
	return Quaternion{ w + other.w, x + other.x, y + other.y, z + other.z };
}
Quaternion Quaternion::operator-(const Quaternion& other) const
{
	return Quaternion{ w - other.w, x - other.x, y - other.y, z - other.z };
}
Quaternion Quaternion::operator*(float scalar) const
{
	return Quaternion{ w * scalar, x * scalar, y * scalar, z * scalar };
}
Quaternion Quaternion::operator/(float scalar) const
{
	return Quaternion{ w / scalar, x / scalar, y / scalar, z / scalar };
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
	return Quaternion{
		w * other.w - x * other.x - y * other.y - z * other.z,
		w * other.x + x * other.w + y * other.z - z * other.y,
		w * other.y - x * other.z + y * other.w + z * other.x,
		w * other.z + x * other.y - y * other.x + z * other.w
	};
}

/**************************** NON-CONST OPERATORS ****************************/
void Quaternion::operator+=(const Quaternion& other)
{
	w += other.w;
	x += other.x;
	y += other.y;
	z += other.z;
}
void Quaternion::operator-=(const Quaternion& other)
{
	w -= other.w;
	x -= other.x;
	y -= other.y;
	z -= other.z;
}
void Quaternion::operator*=(float scalar)
{
	w *= scalar;
	x *= scalar;
	y *= scalar;
	z *= scalar;
}
void Quaternion::operator/=(float scalar)
{
	w /= scalar;
	x /= scalar;
	y /= scalar;
	z /= scalar;
}
