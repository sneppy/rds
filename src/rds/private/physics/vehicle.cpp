#include "physics/vehicle.h"
#include "physics/vehicle-wheel.h"

Vehicle::Vehicle() :
	location(0.f),
	rotation(0.f, vec3::up),
	localTransform(mat4::transform(location, rotation)),
	chassis(),
	wheels(4),
	dragCoefficient(0.5f),
	dragWidth(1.5f),
	dragHeight(1.f),
	dragArea(dragWidth * dragHeight),
	bSimulateGravity(true)
{
	// Setup chassis
	chassis.mass = 1400.f;
	chassis.centerOfMass = vec3(0.f, 0.6f, -0.2f);

	// @todo setup wheels
	const vec3 wheelOffsets[4] = {
		vec3(-1.587 / 2.f, 0.31f, 1.41f),
		vec3(1.587 / 2.f, 0.31f, 1.41f),
		vec3(-1.612 / 2.f, 0.31f, -1.28f),
		vec3(1.612 / 2.f, 0.31f, -1.28f)
	};
	for (uint32 i = 0; i < 4; ++i)
	{
		// Create wheel
		VehicleWheel * wheel = new VehicleWheel;
		wheel->setWheelOffset(wheelOffsets[i]);
		wheel->init(this);
	}
}

void Vehicle::tick(float32 dt)
{	
	/// Compute drag resistance
	/// @todo 
	const vec3 dragResistance	= 0.f;
	const vec3 dragDownforce	= 0.f;
	chassis.applyForce(dragDownforce);

	/// Simulate gravity first
	if (bSimulateGravity) applyGravity(dt);

	// Commit vertical forces
	//chassis.tick(dt);

	/// Simulate physics on each wheel (locally)
	for (auto wheel : wheels)
		wheel->tick(dt);
	
	/// Apply drag resistance
	chassis.applyForce(dragResistance);
	
	/// Run simulation on rigid body
	chassis.tick(dt);

	/// Update vehicle transform
	location = chassis.location;
	rotation = chassis.rotation;
	localTransform = mat4::transform(location, rotation);

	/// Update wheels transform
	for (auto wheel : wheels)
		wheel->postTick(dt);
}