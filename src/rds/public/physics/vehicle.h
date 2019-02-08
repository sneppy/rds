#pragma once

#include "coremin.h"
#include "rigid-body.h"
#include "vehicle-wheel.h"

/**
 * @class Vehicle physics/vehicle.h
 */
class Vehicle
{
protected:
	/// Rigid body attached to this vehicle (the chassis)
	RigidBody chassis;

	/// Wheels attached to the vehicle
	Array<VehicleWheel> wheels;

public:
	/// Default constructor
	Vehicle();
};