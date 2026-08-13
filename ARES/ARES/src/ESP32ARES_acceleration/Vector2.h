#pragma once

#include <iostream>

struct Vector2
{
	float x;
	float y;

public:
	Vector2()
		: x(0.0f), y(0.0f) {}
	Vector2(float val)
		: x(val), y(val) {}
	Vector2(float x, float y)
		: x(x), y(y) {}

	Vector2(const Vector2& other)
		: x(other.x), y(other.y) {}

public:
	float magSq() const;
	float mag() const;
	Vector2 normalised() const;

	float dot(const Vector2& other) const;

public:
	void normalise();
	void setMag(float newMagnitude);
	void clampAxis(float min, float max);

public:
	Vector2 operator+(const Vector2& other) const;
	Vector2 operator-(const Vector2& other) const;
	Vector2 operator*(float scalar) const;
	Vector2 operator/(float scalar) const;

public:
	void operator+=(const Vector2& other);
	void operator-=(const Vector2& other);
	void operator*=(float scalar);
	void operator/=(float scalar);
};

std::ostream& operator<<(std::ostream& os, const Vector2& vec);
