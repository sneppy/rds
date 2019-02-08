#include "physics/vehicle-wheel.h"

void VehicleWheel::tick(float32 dt)
{
	// Get contact point
	vec3 contactPoint;
	bool bGrounded = getContactPoint(contactPoint);

	// Compute suspension force
	suspensionLength = (contactPoint).getSize() - radius;
	const vec3 suspensionForce = getSuspensionForce(dt);
	suspensionPreviousLength = suspensionLength;

	// Calc friction forces
	const float32 maxStaticFriction  = suspensionForce.getSize() * staticFrictionCoefficient;
	const float32 maxKineticFriction = suspensionForce.getSize() * kineticFrictionCoefficient;


}