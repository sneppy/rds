#include "physics/vehicle.h"
#include "physics/vehicle-wheel.h"

Vehicle::Vehicle() :
	location(vec3::up * 0.5f),
	rotation(0.f, vec3::up),
	localTransform(mat4::transform(location, rotation)),
	chassis(),
	wheels(4),
	dragCoefficient(0.32f),
	dragWidth(1.5f),
	dragHeight(1.f),
	dragArea(dragWidth * dragHeight),
	bSimulateGravity(true)
{
	// Setup chassis
	chassis.location = location;
	chassis.rotation = rotation;
	chassis.mass = 1706.f;
	chassis.centerOfMass = vec3(0.f, 0.57f, 0.1f);

	// @todo setup wheels
	const vec3 wheelOffsets[4] = {
		vec3(-1.52f / 2.f, 0.51f, 1.31f),
		vec3(1.52f / 2.f, 0.51f, 1.31f),
		vec3(-1.56f / 2.f, 0.51f, -1.17f),
		vec3(1.56f / 2.f, 0.51f, -1.17f)
	};
	const float32 wheelDistance = (wheelOffsets[0] - wheelOffsets[2]).z;
	for (uint32 i = 0; i < 4; ++i)
	{
		// Create wheel
		VehicleWheel * wheel = new VehicleWheel;
		wheel->setWheelOffset(wheelOffsets[i]);
		wheel->init(this);
		wheel->load = chassis.mass * (1.f - (chassis.centerOfMass - wheelOffsets[i]).z / wheelDistance) / 2.f;
		wheel->bPowered = i > 1;
	}

	wheels[0]->wheelOrientation = wheels[2]->wheelOrientation = quat(M_PI, vec3::up);
}

void Vehicle::tick(float32 dt)
{
	/// Compute drag resistance
	/// @todo 
	const vec3 dragResistance	= dragArea * dragCoefficient * chassis.linearVelocity.getSize() * -chassis.linearVelocity;
	const vec3 dragDownforce	= dragArea * 0.05f * chassis.linearVelocity.getSquaredSize() * chassis.rotation.down();

	/// Simulate gravity first
	if (bSimulateGravity) applyGravity(dt);
	//chassis.applyForce(dragDownforce);
	chassis.tick(dt);

	/// Simulate physics on each wheel (locally)
	for (auto wheel : wheels)
		wheel->tick(dt);

	chassis.accumulatedForce.print();
	
	/// Apply drag resistance
	chassis.applyForce(dragResistance);
	
	/// Run simulation on rigid body
	chassis.tick(dt);

	/// Update vehicle transform
	chassis.location = location += chassis.linearVelocity * dt;
	if (!chassis.angularVelocity.isNearlyZero())
		rotation = chassis.rotation = quat(chassis.angularVelocity.getSize() * dt, chassis.angularVelocity.getNormal()) * rotation;
	
	localTransform = chassis.localTransform = mat4::translation(chassis.location) * mat4::translation(chassis.centerOfMass) * mat4::rotation(chassis.rotation) * mat4::translation(-chassis.centerOfMass);

	printf("speed: %f km/h\n", chassis.linearVelocity.getSize() * 3.6f);

	/// Update wheels transform
	for (auto wheel : wheels)
		wheel->postTick(dt);
}