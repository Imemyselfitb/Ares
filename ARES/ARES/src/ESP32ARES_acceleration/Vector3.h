#pragma once

#include <stdint.h>

extern void OUTPUT_TEXT_ARES(const char* txt);
extern void OUTPUT_FLOAT_ARES(float num, uint8_t dp);

struct Vector3
{
	float x;
	float y;
	float z;

public:
	Vector3()
		: x(0.0f), y(0.0f), z(0.0f) {}
	Vector3(float val)
		: x(val), y(val), z(val) {}
	Vector3(float x, float y, float z)
		: x(x), y(y), z(z) {}

	Vector3(const Vector3& other)
		: x(other.x), y(other.y), z(other.z) {}

public:
	float magSq() const;
	float mag() const;
	Vector3 normalised() const;

	float dot(const Vector3& other) const;
	Vector3 cross(const Vector3& other) const;

	void Print() const;
	void PrintRaw() const;

public:
	void normalise();
	void setMag(float newMagnitude);
	void clampAxis(float min, float max);

public:
	Vector3 operator+(const Vector3& other) const;
	Vector3 operator-(const Vector3& other) const;
	Vector3 operator*(float scalar) const;
	Vector3 operator/(float scalar) const;

public:
	void operator+=(const Vector3& other);
	void operator-=(const Vector3& other);
	void operator*=(float scalar);
	void operator/=(float scalar);
};
