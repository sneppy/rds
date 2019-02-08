#pragma once

#include "coremin.h"

/**
 * @class RigidBody physics/rigid-body
 */
class RigidBody
{
protected:
	/// World space location
	vec3 location;

	/// World space rotation
	quat rotation;

	/// Physics properties
	/// @{
	vec3	centerOfMass;
	float32	mass;
	mat3	inertialTensor;
	/// @}

	/// Linear velocity
	vec3 linearVelocity;

	/// Angular velocity
	vec3 angularVelocity;

	/// Force to be applied to the center of mass
	vec3 totalForce;

	/// Torque to be applied to the center of mass
	vec3 torque;

public:
	/// Default constructor
	RigidBody();

	/// Tick physics
	virtual void tick(float32 dt);

	/// Get point velocity
	FORCE_INLINE vec3 getPointVelocity(const vec3 & point)
	{
		const vec3 r = point - centerOfMass;
		return linearVelocity + (angularVelocity ^ r) / r.getSquaredSize();
	}
};