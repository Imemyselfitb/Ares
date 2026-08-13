#pragma once

#include "Vector2.h"
#include "Vector3.h"
#include "Quaternion.h"

struct PID_Gain
{
	float Kp;
	float Ki;
	float Kd;
};

struct PID_State
{
	Vector3 Target;
	Vector3 TargetOffset;
	Vector3 TargetHeading;
	Vector2 ServoOrientation;
	float Thrust;
};

class PIDSystem
{
public:
	PIDSystem() {}

public:
	Vector3 getTargetOrientation(float delta, const Vector3& offsetToTarget, const Vector3& velocity, float percentMassRemaining);
	Vector2 orientateThruster(float delta, const Vector3& targetHeading, const Quaternion& orientation, const Vector3& angVel);
	float throttleThrust(float delta, float yTargetOffset, float yVelocity);

private:
	PID_Gain m_PositionPID{ 1.2f, 0.4f, 9.5f };
	PID_Gain m_OrientationPID{ 0.65f, 0.25f, 0.8f };
	PID_Gain m_ThrusterPID{ 1.0f, 0.1f, 1.8f };

private:
	Vector2 m_SumErrorOrientation{};
	Vector2 m_SumErrorPosition{};
	float m_SumErrorThrust{};
};
