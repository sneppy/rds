#pragma once

#include "coremin.h"

/// @todo delete
extern float32 carThrottle;
extern float32 carBrake;

/// Forward decls
class Vehicle;

/**
 * @class VehicleWheel
 */
class VehicleWheel
{
public:
	/// Vehicle that owns this wheel
	Vehicle * vehicle;

	/// Wheel local transform
	/// @{
	vec3 localOffset;
	quat localRotation;
	mat4 localTransform;
	/// @}

	/// Suspension join position
	vec3 jointOffset;

	/// Wheel mass
	float32 mass;

	/// Load of the car on this wheel
	float32 load;

	/// Wether this wheel receives power from engine
	bool bPowered;

	/// Suspension model
	/// @{
	float32 suspensionLength;
	float32 suspensionPreviousLength;
	float32 suspensionStiffness;
	float32 suspensionDamping;
	float32 suspensionRestLength;
	/// @}

	/// Geometric properties
	/// @{
	float32 radius;
	/// @}

	/// Friction values
	/// @{
	float32 staticFrictionCoefficient;
	float32 kineticFrictionCoefficient;
	float32 rollingFrictionCoefficient;
	/// @}
	
	/// Steering angle (radians)
	float32 steeringAngle;

	//////////////////////////////////////////////////
	// Visuals only
	//////////////////////////////////////////////////

	/// Wheel orientation
	quat wheelOrientation;

	/// Wheel rolling speed and phase
	/// @{
	float32 rollingSpeed;
	float32 rollingPhase;
	/// @}

public:
	/// Default constructor
	VehicleWheel();

	/// Init wheel
	void init(Vehicle * owner);

protected:
	/// Returns ground contact point
	bool getContactPoint(vec3 & contactPoint);

	/// Computes applied suspension force
	FORCE_INLINE vec3 getSuspensionForce(float32 dt) const
	{
		const float32 suspensionCompression	= suspensionRestLength - suspensionLength;
		const float32 suspensionVelocity	= (suspensionLength - suspensionPreviousLength) / dt;

		const float32 suspensionForce
		= suspensionStiffness * suspensionCompression	// Spring force
		- suspensionDamping * suspensionVelocity;		// Drag force

		return vec3::up * suspensionForce;
	}

	/// Computes the drive applied to the wheel
	FORCE_INLINE float32 getWheelDrive();

public:
	/// Set wheel offset w.r.t chassis
	void setWheelOffset(const vec3 offset)
	{
		jointOffset = offset;
	}

	/// Tick physics
	void tick(float32 dt);

	/// Called after vehicle physics has been updated
	void postTick(float32 dt);
};