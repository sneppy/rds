#include "physics/rigid-body.h"

RigidBody::RigidBody() :
	location(0.f),
	rotation(0.f, vec3::up),
	localTransform(mat4::transform(location, rotation)),
	mass(10.f),
	inertiaTensor(), // @todo
	centerOfMass(location),
	linearVelocity(0.f),
	angularVelocity(0.f),
	accumulatedForce(0.f),
	accumulatedTorque(0.f)
{
	// @todo maybe calculate inertia tensor somehow?
}

void RigidBody::tick(float32 dt)
{
	// Compute accelerations
	const vec3 linearAcceleration	= accumulatedForce / mass;
	const vec3 angularAcceleration	= accumulatedTorque / ((1.f / 12.f) * mass * (25.f)); // @todo this should be inertia tensor

	printf("a: %f\n", linearAcceleration.getSize());

	// Integrate to find new velocity
	linearVelocity	+= linearAcceleration * dt;
	angularVelocity += angularAcceleration * dt;
	const float32 angularSpeed = angularVelocity.getSize();

	// Integrate velocity to find new location and rotation
	/* location += linearVelocity * dt;
	if (angularSpeed > FLT_EPSILON)
	{
		rotation  = quat(angularSpeed * dt, angularVelocity / angularSpeed) * rotation;
		rotation.normalize();
	}

	// Update local transform
	localTransform = mat4::transform(location, rotation); */

	// Reset accumulated forces
	accumulatedForce	= 0.f;
	accumulatedTorque	= 0.f;
}