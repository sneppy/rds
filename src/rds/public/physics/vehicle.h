#pragma once

#include "coremin.h"
#include "rigid-body.h"

/// Forward declare wheel class
class VehicleWheel;

/**
 * @class Vehicle physics/vehicle.h
 */
class Vehicle
{
	friend VehicleWheel;

public:
	/// Vehicle world space transform
	/// @{
	vec3 location;
	quat rotation;
	mat4 localTransform;
	/// @}

	/// Rigid body attached to this vehicle (the chassis)
	RigidBody chassis;

	/// Wheels attached to this vehicle
	Array<VehicleWheel*> wheels;

	/// Car aerodynamics
	/// @{
	float32 dragCoefficient;
	float32 dragArea;
	float32 dragHeight;
	float32 dragWidth;
	/// @{

	/// Flag to enable/disable gravity
	bool bSimulateGravity;

public:
	/// Default constructor
	Vehicle();

	/// Tick vehicle (input and physics)
	virtual void tick(float32 dt);

	/// Apply gravity (called before any other physics related stuff)
	FORCE_INLINE void applyGravity(float32 dt)
	{
		// @todo gravity value should be a world property
		const vec3 gravityAcceleration = vec3::down * 9.81f;
		chassis.applyForce(chassis.mass * gravityAcceleration); // N = mg
	}
};