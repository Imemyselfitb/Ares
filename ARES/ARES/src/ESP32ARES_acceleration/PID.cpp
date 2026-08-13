#include "PID.h"

#define PI 3.14159265f

Vector3 PIDSystem::getTargetOrientation(float delta, const Vector3& targetOffset, const Vector3& velocity, float percentMassRemaining)
{
	Vector2 error{ targetOffset.x, targetOffset.z };
	Vector2 vel{ velocity.x, velocity.z };

	Vector2 targetAccel = (error * m_PositionPID.Kp) +
		(m_SumErrorPosition * m_PositionPID.Ki) -
		(vel * m_PositionPID.Kd);

	m_SumErrorPosition += error * delta;
	m_SumErrorPosition *= 1.0f - 0.0001f;

	targetAccel *= percentMassRemaining;

	Vector3 targetHeading{ targetAccel.x, 25.0f, targetAccel.y };
	targetHeading.normalise();
	return targetHeading;
}

Vector2 PIDSystem::orientateThruster(float delta, const Vector3& targetHeading, const Quaternion& orientation, const Vector3& angVel)
{
	Quaternion up = Quaternion{ Vector3{0.0f, 1.0f, 0.0f} };
	Vector3 currentForward = (orientation * up * orientation.conjugate()).getVector();

	Quaternion attitudeQ = Quaternion{ currentForward.cross(targetHeading) };
	Vector3 attitude = (orientation.conjugate() * attitudeQ * orientation).getVector();

	Vector2 error{ attitude.x, attitude.z };

	Quaternion angVelQ{ angVel };
	Vector3 angVelBody = (orientation.conjugate() * angVelQ * orientation).getVector();
	Vector2 angV{ angVelBody.x, angVelBody.z };

	Vector2 servoOutput = (error * m_OrientationPID.Kp) +
		(m_SumErrorOrientation * m_OrientationPID.Ki) -
		(angV * m_OrientationPID.Kd);
	
	servoOutput.clampAxis(-PI * 0.25f, PI * 0.25f);

	m_SumErrorOrientation += error * delta;
	m_SumErrorOrientation *= 1.0f - 0.0001f;

	return servoOutput;
}

float PIDSystem::throttleThrust(float delta, float yTargetOffset, float yVelocity)
{
	float throttle = (yTargetOffset * m_ThrusterPID.Kp) +
		(m_SumErrorThrust * m_ThrusterPID.Ki) -
		(yVelocity * m_ThrusterPID.Kd);

	m_SumErrorThrust += yTargetOffset * delta;
	m_SumErrorThrust *= 1.0f - 0.0001f;

	return throttle;
}
