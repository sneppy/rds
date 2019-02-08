#pragma once

#include "coremin.h"

/**
 * @class VehicleWheel
 */
class VehicleWheel
{
protected:
	/// Wheel mass
	float32 mass;

	/// Load of the car on this wheel
	float32 load;

	/// Suspension model
	/// @{
	float32 suspensionLength;
	float32 suspensionRestLength;
	float32 suspensionPreviousLength;
	float32 suspensionStiffness;
	float32 suspensionDamping;
	/// @}

	/// Geometric properties
	/// @{
	float32 radius;
	/// @}

	/// Friction values
	/// @{
	float32 staticFrictionCoefficient;
	float32 kineticFrictionCoefficient;
	/// @}

public:
	/// Default constructor
	VehicleWheel();

protected:
	/// Returns ground contact point
	FORCE_INLINE bool getContactPoint(vec3 & contactPoint)
	{
		// @todo raycast and find hit point
		contactPoint = vec3::zero;
		return true;
	}

	/// Computes applied suspension force
	FORCE_INLINE vec3 getSuspensionForce(float32 dt) const
	{
		const float32 suspensionCompression	= suspensionRestLength - suspensionLength;
		const float32 suspensionVelocity	= (suspensionPreviousLength - suspensionLength) / dt;

		const float32 suspensionForce
		= suspensionStiffness * suspensionCompression	// Spring force
		+ suspensionDamping * suspensionVelocity;		// Drag force

		return vec3::up * suspensionForce;
	}

	/// Computes the drive applied to the wheel
	FORCE_INLINE float32 getWheelDrive();

public:
	/// Tick physics
	void tick(float32 dt);
};