#pragma once

#include "coremin.h"

/**
 * @class RigidBody physics/rigid-body
 */
class RigidBody
{
public:
	/// World space location
	vec3 location;

	/// World space rotation
	quat rotation;

	/// Local transform
	mat4 localTransform;

	/// Physics properties
	/// @{
	float32	mass;
	mat3	inertiaTensor;
	vec3	centerOfMass;
	/// @}

	/// Linear velocity
	vec3 linearVelocity;

	/// Angular velocity
	vec3 angularVelocity;

	/// Force to be applied to the center of mass
	vec3 accumulatedForce;

	/// Torque to be applied to the center of mass
	vec3 accumulatedTorque;

public:
	/// Default constructor
	RigidBody();

	/// Get point velocity
	FORCE_INLINE vec3 getPointVelocity(const vec3 & point) const
	{
		const vec3 r = point - (localTransform * centerOfMass);
		return linearVelocity + (angularVelocity ^ r);
	}

	/// Apply force at point
	FORCE_INLINE void applyForce(const vec3 & force, const vec3 & point)
	{
		// Forces contribute to accumulated force
		// plus accumulated torque
		accumulatedForce  += force;
		accumulatedTorque += (point - (localTransform * centerOfMass)) ^ force;
	}

	/// Apply force at center of mass
	FORCE_INLINE void applyForce(const vec3 & force)
	{
		// No torque generated
		accumulatedForce  += force;
	}

	/// Tick physics
	virtual void tick(float32 dt);
};