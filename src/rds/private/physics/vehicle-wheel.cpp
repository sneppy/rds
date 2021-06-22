#include "physics/vehicle-wheel.h"
#include "physics/vehicle.h"

VehicleWheel::VehicleWheel() :
	vehicle(nullptr),
	localOffset(0.f),
	localRotation(0.f, vec3::up),
	localTransform(mat4::transform(localOffset, quat(0.f, vec3::up))),
	jointOffset(0.f),
	mass(15.f), // Kg
	load(350.f),
	bPowered(true),
	suspensionLength(0.21f),
	suspensionPreviousLength(suspensionLength),
	suspensionStiffness(86000.f),
	suspensionDamping(9000.f),
	suspensionRestLength((load * 9.81f) / suspensionStiffness + suspensionLength), // @todo gravity property of the world
	radius(0.31f),
	staticFrictionCoefficient(1.2f),
	kineticFrictionCoefficient(1.1f),
	rollingFrictionCoefficient(0.03f),
	steeringAngle(0.f),
	rollingSpeed(0.f),
	rollingPhase(0.f) {}

void VehicleWheel::init(Vehicle * owner)
{
	// Register and add to vehicle
	vehicle = owner;
	vehicle->wheels.push(this);
}

bool VehicleWheel::getContactPoint(vec3 & contactPoint)
{
	// @todo raycast and find hit point
	/* contactPoint = vehicle->localTransform * jointOffset;
	contactPoint.y = 0.f;
	return true; */

	vec3 dir = vehicle->rotation.down();
	vec3 p = vehicle->localTransform * jointOffset;
	float32 t = -p.y / dir.y;

	contactPoint = p + t * dir;
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

	printf("%f\n", suspensionLength);
	printf("%f\n", suspensionForce.y);

	vehicle->chassis.applyForce(suspensionForce, contactPoint); // @todo at wheel contact point or suspension joint

	// Calc friction forces
	load = suspensionForce.getSize();
	const float32 maxStaticFriction  = load * staticFrictionCoefficient;
	const float32 maxKineticFriction = load * kineticFrictionCoefficient;
	const float32 maxRollingFriction = load * rollingFrictionCoefficient;

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
	const vec3 tangentialMomentum = tangentialVelocity * (load / 9.81f + mass);
	vec3 sideForce = (-tangentialMomentum & right) * right / dt;

	// Set rolling speed if no forces
	// Just for visuals
	rollingSpeed = (vehicle->chassis.linearVelocity & forward) / radius;

	// Forward force
	vec3 rollingForce;
	if (carThrottle > carBrake & bPowered)
	{
		// @todo motor force should be determined by engine -> wheel torque
		const float32 motorForce = 55600.f * carThrottle;
		rollingForce = forward * motorForce;
	}
	else
	{
		// What's the impulse that would immediately stop the car?
		// @todo compute that value
		const vec3 stopImpulse = (-vehicle->chassis.linearVelocity * vec3(1.f, 0.f, 1.f)) * (vehicle->chassis.mass / 4.f) / dt;

		// Applied break force
		const float32 brakeForce = carBrake * 19600.f;

		if (brakeForce < 29600.f)
		{		
			// compute stop force
			const float32 rollingResistance = maxRollingFriction;
			const vec3 stopForce = (rollingResistance + brakeForce) * (-vehicle->chassis.linearVelocity.getSafeNormal() & forward) * forward;

			rollingForce = stopForce.getSize() < stopImpulse.getSize() ? stopForce : stopImpulse;
		}
		else
		{
			// No side force here
			sideForce = 0.f;
			rollingSpeed = 0.f;

			const vec3 stopForce = brakeForce * (-vehicle->chassis.getPointVelocity(contactPoint) * vec3(1.f, 0.f, 1.f)).getSafeNormal();
			rollingForce = stopForce.getSize() < stopImpulse.getSize() ? stopForce : stopImpulse;
		}		
	}

	// Compute the sum of forward and lateral force
	vec3 groundForce = sideForce + rollingForce;
	const float32 groundForceSize = groundForce.getSize();

	printf("%f\n", maxStaticFriction);

	if (groundForceSize > maxStaticFriction)
	{
		// Limit forward and side force by the max static friction
		const float32 factor = maxStaticFriction / groundForceSize;
		groundForce		*= maxStaticFriction / groundForceSize;
		rollingForce	*= factor,
		sideForce		*= factor;
	}

	// Apply forward and side forces
	vehicle->chassis.applyForce(groundForce, contactPoint);
	//vehicle->chassis.applyForce(rollingForce, contactPoint);
	//vehicle->chassis.applyForce(sideForce, contactPoint);
}

void VehicleWheel::postTick(float32 dt)
{
	// Here we should update wheel transform
	// only for rendering purpose
	rollingPhase += rollingSpeed * dt;

	localOffset		= jointOffset - vec3::up * Math::min(suspensionRestLength, suspensionLength);
	localRotation	= quat(steeringAngle, vec3::up) * quat(rollingPhase, vec3::right) * wheelOrientation;
	localTransform	= mat4::transform(localOffset, localRotation);
}