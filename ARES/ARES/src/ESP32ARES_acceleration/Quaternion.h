#pragma once

#include "Vector3.h"

#include <stdint.h>

extern void OUTPUT_TEXT_ARES(const char* txt);
extern void OUTPUT_FLOAT_ARES(float num, uint8_t dp);

struct Quaternion
{
	float w = 1.0;
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;

public:
	Quaternion() {}
	Quaternion(float w, float x, float y, float z)
		: w(w), x(x), y(y), z(z) {}
	Quaternion(const Vector3& vec3, float w = 0.0)
		: w(w), x(vec3.x), y(vec3.y), z(vec3.z) {}

	Quaternion(const Vector3& from, const Vector3& to);

	Quaternion(const Quaternion& other)
		: w(other.w), x(other.x), y(other.y), z(other.z) {}

public:
	float magSq() const;
	float mag() const;
	Quaternion normalised() const;
	Quaternion conjugate() const;
	Quaternion inverse() const;

	Vector3 getVector() const;
	Vector3 rotateVector(const Vector3& point) const;

	void toRotationMatrix(float* matrix) const;

	void Print() const;

public:
	Quaternion operator+(const Quaternion& other) const;
	Quaternion operator-(const Quaternion& other) const;
	Quaternion operator*(float scalar) const;
	Quaternion operator/(float scalar) const;

	Quaternion operator*(const Quaternion& other) const;

public:
	void operator+=(const Quaternion& other);
	void operator-=(const Quaternion& other);
	void operator*=(float scalar);
	void operator/=(float scalar);
};
