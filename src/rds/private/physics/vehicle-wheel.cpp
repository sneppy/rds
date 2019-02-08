#include "physics/vehicle-wheel.h"
#include "physics/vehicle.h"

VehicleWheel::VehicleWheel() :
	vehicle(nullptr),
	localOffset(0.f),
	localRotation(0.f),
	localTransform(mat4::transform(localOffset, quat(0.f, vec3::up))),
	jointOffset(0.f),
	mass(15.f), // Kg
	load(350.f),
	suspensionLength(0.f),
	suspensionPreviousLength(suspensionLength),
	suspensionStiffness(50000.f),
	suspensionDamping(3000.f),
	suspensionRestLength((load * 9.81f) / suspensionStiffness + suspensionLength), // @todo gravity property of the world
	radius(0.31f),
	staticFrictionCoefficient(1.f),
	kineticFrictionCoefficient(0.8f),
	rollingFrictionCoefficient(0.014f) {}

void VehicleWheel::init(Vehicle * owner)
{
	// Register and add to vehicle
	vehicle = owner;
	vehicle->wheels.push(this);
}

bool VehicleWheel::getContactPoint(vec3 & contactPoint)
{
	// @todo raycast and find hit point
	contactPoint = vehicle->localTransform * jointOffset;
	contactPoint.y = 0.f;
	return true;
}

void VehicleWheel::tick(float32 dt)
{
	// Get contact point
	vec3 contactPoint;
	bool bGrounded = getContactPoint(contactPoint);

	// Compute suspension force
	suspensionLength = (contactPoint - vehicle->chassis.localTransform * jointOffset).getSize() - radius;
	const vec3 suspensionForce = getSuspensionForce(dt);
	suspensionPreviousLength = suspensionLength;

	vehicle->chassis.applyForce(suspensionForce, contactPoint); // @todo at wheel contact point or suspension joint

	// Calc friction forces
	const float32 maxStaticFriction  = suspensionForce.getSize() * staticFrictionCoefficient;
	const float32 maxKineticFriction = suspensionForce.getSize() * kineticFrictionCoefficient;

	// Get direction vectors
	// @todo use wheel steering
	const quat steering = vehicle->chassis.rotation * quat(steeringAngle, vehicle->chassis.rotation.up());
	const vec3
		forward	= steering.forward(),
		right	= steering.right(),
		up		= steering.up();

	// Side force
	// @todo there's a better way to compute impulse
	// For now, we compute the velocity of the wheel
	// onto the orthogonal axis, and apply and impulse
	// that would cancel out this velocity
	const vec3 tangentialVelocity = vehicle->chassis.getPointVelocity(contactPoint);
	const vec3 tangentialMomentum = load * tangentialVelocity;
	vec3 sideForce = (-tangentialMomentum & right) * right / dt;

	// Forwawrd force
	vec3 rollingForce;
	if (true/* motor force is on */)
	{
		// @todo motor force should be determined by engine -> wheel torque
		const float32 motorForce = 2000.f * carThrottle;
		rollingForce = forward * motorForce;
	}
	else
	{
		// What's the impulse that would immediately stop the car?
		// @todo compute that value
		float32 stopImpulse = 0.f;
		
		// The max deceleration is limited by the max friction force
		const float32 rollingResistance	= suspensionForce.getSize() * rollingFrictionCoefficient;
		const float32 brakeForce		= 0.f;
		const float32 stopForce			= Math::min(stopImpulse, Math::max(brakeForce, rollingResistance));

		rollingForce = (forward & vehicle->chassis.linearVelocity) > 0.f ? -stopForce : stopForce;
	}

	// Compute the sum of forward and lateral force
	vec3 groundForce = sideForce + rollingForce;
	const float32 groundForceSize = groundForce.getSize();

	if (groundForceSize > maxStaticFriction)
	{
		// Limit forward and side force by the max static friction
		const float32 factor = maxStaticFriction / groundForceSize;
		rollingForce	*= factor,
		sideForce		*= factor;
	}

	// Apply forward and side forces
	vehicle->chassis.applyForce(rollingForce, contactPoint);
	vehicle->chassis.applyForce(sideForce, contactPoint);
}

void VehicleWheel::postTick(float32 dt)
{
	// Here we should update wheel transform
	// only for rendering purpose
	localOffset		= jointOffset - vec3::up * suspensionLength;
	localTransform	= mat4::transform(localOffset, quat(steeringAngle, vec3::up));
}