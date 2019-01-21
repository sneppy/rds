#include "coremin.h"
#include "gldrv/gldrv.h"
#include "primitives/box.h"
#include <SDL.h>

Malloc * gMalloc = nullptr;

ansichar * readFile(const String & filename);
uint32 createProgram();

vec3 cameraLocation;
vec3 cameraSpeed;
quat cameraRotation;
mat4 viewProjectionMatrix;

const float32
	accelerationFactor = 16.f,
	brakeFactor = 4.f;

class Wheel
{
public:
	/// Wheel relative transform
	vec3 offset;
	quat orientation;
	quat phase;

	/// Geometric properties
	float32 radius;

	/// Physics properties
	float32 mass;
	float32 inertiaMoment;
	float32 staticCoeff;
	float32 kineticCoeff;
	float32 tirePressure;

	/// Vehicle properties
	uint8 bDrive;

	/// Dynamics
	vec3 linearMomentum;
	vec3 linearVelocity;
	vec3 angularMomentum;
	vec3 angularVelocity;

	/// User input
	float32 steeringAngle;
	float32 throttle;

	/// Owning vehicle
	class Vehicle * vehicle;

public:
	/// Default constructor
	Wheel(vec3 _offset = 0.f) :
		offset(_offset),
		orientation(0.f, vec3::up),
		phase(M_PI, vec3::right),
		radius(0.21f),
		mass(20.f),
		inertiaMoment(mass * radius * radius / 2.f),
		staticCoeff(1.f),
		kineticCoeff(0.8f),
		tirePressure(2.1f),
		linearMomentum(0.f),
		linearVelocity(linearMomentum / mass),
		angularMomentum(0.f),
		angularVelocity(angularMomentum / inertiaMoment),
		steeringAngle(0.f),
		vehicle(nullptr) {}
	
	FORCE_INLINE void tickPhysics(float32 dt);
};
	
class Vehicle
{
	friend Wheel;

public:
	/// Car transform
	vec3 location;
	quat rotation;

	/// Car properties
	Array<Wheel> wheels;

	/// Physics properties
	float32 mass;
	float32 inertiaMoment;
	vec3 centerOfMass;

	/// Dynamics
	vec3 linearMomentum;
	vec3 linearVelocity;
	vec3 angularMomentum;
	vec3 angularVelocity;
	vec3 wheelsForce;
	vec3 wheelsTorque;

public:
	/// Default constructor
	Vehicle() :
		location(0.f),
		rotation(0.f, vec3::up),
		wheels(4),
		mass(1500.f),
		inertiaMoment((1.f / 12.f) * mass * (4.f + 18.f)),
		centerOfMass(0.f, 0.f, -0.2f),
		linearMomentum(0.f),
		linearVelocity(0.f),
		angularMomentum(0.f),
		angularVelocity(0.f),
		wheelsForce(0.f),
		wheelsTorque(0.f) {}
	
	/// Setup wheels
	FORCE_INLINE void setupWheels()
	{
		wheels(0) = Wheel(vec3(-1.587 / 2.f, 0.f, 1.4f)); // Left
		wheels(1) = Wheel(vec3(1.587 / 2.f, 0.f, 1.4f)); // Left
		wheels(2) = Wheel(vec3(-1.587 / 2.f, 0.f, -1.f)); // Left
		wheels(3) = Wheel(vec3(1.587 / 2.f, 0.f, -1.f)); // Left

		wheels[0].vehicle = this;
		wheels[1].vehicle = this;
		wheels[2].vehicle = this;
		wheels[3].vehicle = this;
	}

	/// Set throttle
	FORCE_INLINE void setThrottle(float32 value)
	{
		wheels[0].throttle = value;
		wheels[1].throttle = value;
		wheels[2].throttle = value;
		wheels[3].throttle = value;
	}

	/// Set steering on wheels
	FORCE_INLINE void setSteering(float32 value)
	{
		const float32
			axleRadius = 0.2f,
			axleMaxLength = 1.2f,
			axleMinLength = 1.f;
		
		const float32
			steeringAngle = value * 0.2f,
			alpha = M_PI / 3.f + Math::abs(steeringAngle),
			A = Math::sin(alpha),
			B = (axleMaxLength / axleRadius) - Math::cos(alpha),
			C = 1.f - (axleMaxLength / axleRadius) * Math::cos(alpha) + (axleMaxLength * axleMaxLength - axleMinLength * axleMinLength) / (2.f * axleRadius * axleRadius),
			resultAngle = 2.f * Math::atan((A + Math::sqrt(A * A + B * B - C * C)) / (B + C));
		
		const float32
			leftWheelAngle	= value > 0.f ? steeringAngle : -M_PI / 3.f + resultAngle,
			rightWheelAngle	= value > 0.f ? M_PI / 3.f - resultAngle : steeringAngle;
		
		wheels[0].steeringAngle = Math::lerp(wheels[0].steeringAngle, leftWheelAngle, 0.1f),
		wheels[1].steeringAngle = Math::lerp(wheels[1].steeringAngle, rightWheelAngle, 0.1f);
	}

	/// Simulate physics
	FORCE_INLINE void tickPhysics(float32 dt)
	{
		// Delegate to wheels
		for (auto & wheel : wheels)
			wheel.tickPhysics(dt);

		// Update momentum
		linearMomentum	+= (rotation * wheelsForce) * dt;
		angularMomentum	+= wheelsTorque * dt;

		linearVelocity	= linearMomentum / mass;
		angularVelocity	= angularMomentum / inertiaMoment;

		// Update location
		location += linearVelocity * dt;
		if (!angularVelocity.isNearlyZero())
			rotation = quat(angularVelocity.getSize() * dt, angularVelocity.getNormal()) * rotation,
			rotation.normalize();
		
		wheelsForce		= 0.f;
		wheelsTorque	= 0.f;
	}

	FORCE_INLINE void applyForce(const vec3 & f, const vec3 & p)
	{
		wheelsForce += f;
		wheelsTorque += (p - centerOfMass) ^ f;
	}

	FORCE_INLINE Array<mat4> getWheelsTransforms() const
	{
		Array<mat4> transforms;
		mat4 vehicleTransform = mat4::transform(location, rotation);
		for (const auto & wheel : wheels)
			transforms += vehicleTransform * mat4::transform(wheel.offset, wheel.orientation, vec3(wheel.radius));
		
		return transforms;
	}
};

FORCE_INLINE void Wheel::tickPhysics(float32 dt)
{
	if (vehicle == nullptr) return;

	// Compute suspension force
	// Static for now
	const vec3 suspensionForce = vec3::up * ((vehicle->mass) / 4.f + this->mass) * 9.81f; // Gravity force

	// Static and knetic friction forces
	const float32
		staticFriction	= staticCoeff * suspensionForce.getSize(),
		kineticFriction	= kineticCoeff * suspensionForce.getSize();
	
	// Wheel torque
	const quat steeringOrientation = quat(steeringAngle, vec3::up);
	const vec3 tractiveForce = steeringOrientation.forward() * 1000.f * throttle;

	// Rolling resistance
	const vec3 rollingResistance = -linearVelocity.getSafeNormal() * (0.005f + (1.f / tirePressure) * (0.01f + (linearVelocity * linearVelocity).getSize() / 771.f)) * suspensionForce.getSize();

	// Distributed drag resistance
	const vec3 dragResistance = -(!vehicle->rotation * vehicle->linearVelocity) * vehicle->linearVelocity.getSize() * 0.45f / 4.f;

	// Forward force
	const vec3 forwardForce = tractiveForce + rollingResistance + dragResistance;
	
	// Lateral force
	const vec3 lateralLinearMotion = ((!vehicle->rotation * vehicle->linearMomentum / 4.f) & steeringOrientation.right()) * steeringOrientation.right();
	const vec3 centerOffset = (offset - vehicle->centerOfMass);
	const vec3 lateralAngularMotion = (vehicle->angularMomentum ^ centerOffset) / (4.f * centerOffset.getSize() * centerOffset.getSize());
	const vec3 lateralForce = -(lateralLinearMotion + lateralAngularMotion) / dt;

	// Check friction
	vec3 groundForce = lateralForce + forwardForce;
	if (groundForce.getSize() > staticFriction + dragResistance.getSize())
	{
		printf("too much! %f\n", groundForce.getSize());
		groundForce *= kineticFriction / groundForce.getSize();
	}
	
	// Apply force to chassis
	vehicle->applyForce(groundForce, offset);

	// Update wheel rotation
	angularVelocity = vec3::right * vehicle->linearVelocity.getSize() / radius;
	vehicle->linearVelocity.print();
	if (!angularVelocity.isNearlyZero())
		phase = quat(angularVelocity.getSize() * dt, angularVelocity.getNormal()) * phase;

	// Show wheel orientation
	orientation = steeringOrientation * phase;
}

int main()
{
	Memory::createGMalloc();
	srand(clock());

	// Setup input
	Map<uint32, int32> keys;
	point2 cursorLocation, cursorDelta;

	// Setup car
	Vehicle car;
	car.setupWheels();

	// Init keys
	keys[SDLK_w] = 0,
	keys[SDLK_a] = 0,
	keys[SDLK_s] = 0,
	keys[SDLK_d] = 0,
	keys[SDLK_SPACE] = 0,
	keys[SDLK_LCTRL] = 0;

	bool bShouldQuit = false;

	initOpenGL();

	SDL_Window * window = SDL_CreateWindow("rds", 0, 0, 1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_BORDERLESS);
	SDL_GLContext context = SDL_GL_CreateContext(window);

	SDL_WarpMouseInWindow(window, 1920 / 2, 1080 / 2);

	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Setup a nice little cube
	using Vertex = Vec3<float32, false>;
	Array<Vertex> vertices;
	vertices += Vertex(-1.f, -1.f, -1.f);
	vertices += Vertex(-1.f, -1.f, 1.f);
	vertices += Vertex(-1.f, 1.f, -1.f);
	vertices += Vertex(-1.f, 1.f, 1.f);
	vertices += Vertex(1.f, -1.f, -1.f);
	vertices += Vertex(1.f, -1.f, 1.f);
	vertices += Vertex(1.f, 1.f, -1.f);
	vertices += Vertex(1.f, 1.f, 1.f);

	Array<dim3> triangles;
	triangles.push((const dim3[]){
		dim3(1, 0, 3),
		dim3(0, 2, 3),
		dim3(0, 4, 2),
		dim3(4, 6, 2),
		dim3(4, 5, 6),
		dim3(5, 7, 6),
		dim3(5, 1, 7),
		dim3(1, 3, 7),
		dim3(2, 6, 3),
		dim3(6, 7, 3),
		dim3(1, 5, 0),
		dim3(5, 4, 0)
	}, 12);

	printf("OpenGL version is (%s)\n", glGetString(GL_VERSION));

	// Setup cube in opengl
	uint32 prog = createProgram();
	glUseProgram(prog);

	uint32 vao, vbo, ebo;
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	glGenBuffers(1, &ebo);

	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.getCount() * sizeof(Vertex), *vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangles.getCount() * sizeof(dim3), *triangles, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	// Setup camera and model transform
	Map<String, uint32> uniforms;
	uniforms["modelMatrix"] = glGetUniformLocation(prog, "modelMatrix"),
	uniforms["viewProjectionMatrix"] = glGetUniformLocation(prog, "viewProjectionMatrix");

	mat4 modelMatrix = mat4::translation(0.f, 0.f, 1.f) * mat4::rotation(M_PI_4, vec3::unit);
	viewProjectionMatrix = mat4::glProjection(M_PI_2, 1.f) * mat4::translation(vec3(0.f, 0.f, 3.f));

	// Time variables
	uint64 currTick = SDL_GetPerformanceCounter();
	uint64 lastTick = currTick;
	float32 dt = 0.f;

	// Positions of reference cubes
	Array<mat4> cubes(128);
	for (uint8 i = 0; i < cubes.getSize(); ++i)
		cubes += mat4::translation(vec3(Math::randf(), 0.f, Math::randf()) * 100.f);

	// Camera setup
	cameraLocation = vec3(0.f, 1.f, -5.f);
	cameraRotation = quat(0.f, vec3::right);

	while (!bShouldQuit)
	{
		// Update delta time
		currTick = SDL_GetPerformanceCounter();
		dt = (float32)(currTick - lastTick) / SDL_GetPerformanceFrequency();
		lastTick = currTick;

		//printf("FPS: %u\n", (uint32)(1.f / dt));
		
		SDL_Event event;
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_QUIT:
					bShouldQuit = true;
					break;
				case SDL_KEYDOWN:
					keys[(uint32)event.key.keysym.sym] = 1;
					break;
				case SDL_KEYUP:
					keys[(uint32)event.key.keysym.sym] = 0;
					break;
			}
		}

		// Handle input
		const vec3 cameraDirection(cameraRotation.forward());
		const float32
			movementX = keys[SDLK_d] - keys[SDLK_a],
			movementZ = keys[SDLK_w] - keys[SDLK_s],
			movementY = keys[SDLK_SPACE] - keys[SDLK_LSHIFT];
		
		// Camera location
		vec3 cameraAcceleration(movementX, movementY, movementZ);
		cameraSpeed += ((/* cameraRotation *  */cameraAcceleration) * accelerationFactor - cameraSpeed * brakeFactor) * dt;
		cameraLocation += cameraSpeed * dt;

		cameraLocation = Math::lerp(cameraLocation, car.location + (car.rotation.backward() + vec3::up * 0.3f) * 10.f, 0.8f);
		cameraRotation = car.rotation;

		car.setThrottle((float32)keys[SDLK_UP]);
		car.setSteering((float32)(keys[SDLK_RIGHT] - keys[SDLK_LEFT]));

		// Simulate physics
		car.tickPhysics(dt);

		// Draw
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		viewProjectionMatrix = mat4::glProjection(M_PI_2, 0.5f) * mat4::rotation(!cameraRotation) * mat4::translation(-cameraLocation);
		glUniformMatrix4fv(uniforms["viewProjectionMatrix"], 1, GL_TRUE, viewProjectionMatrix.array);

		// Draw car
		for (const auto wheelTransform : car.getWheelsTransforms())
		{
			glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, wheelTransform.array);
			glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_INT, 0);
		}
		/* glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, mat4::transform(car.location, car.rotation, vec3(1.f)).array);
		glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_INT, 0); */

		/// Reference cubes
		for (const auto & cube : cubes)
		{
			glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, cube.array);
			glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_INT, 0);
		}

		SDL_GL_SwapWindow(window);
	}
}

ansichar * readFile(const String & filename)
{
	FILE * fp = fopen(*filename, "r");
	if (!fp) return nullptr;

	// Get file size
	fseek(fp, 0, SEEK_END);
	uint64 size = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	ansichar * buffer = reinterpret_cast<ansichar*>(gMalloc->malloc(size + 1));
	fread(buffer, 1, size, fp);
	buffer[size] = '\0';

	fclose(fp);
	return buffer;
}

uint32 createProgram()
{
	uint32 prog = glCreateProgram();

	// Create shaders
	uint32
		vShader = glCreateShader(GL_VERTEX_SHADER),
		fShader = glCreateShader(GL_FRAGMENT_SHADER);
	
	ansichar
		* vShaderSource = readFile("assets/shaders/default/.vert.glsl"),
		* fShaderSource = readFile("assets/shaders/default/.frag.glsl");
	
	glShaderSource(vShader, 1, &vShaderSource, nullptr),
	glShaderSource(fShader, 1, &fShaderSource, nullptr);
	glCompileShader(vShader), glCompileShader(fShader);

	/* int32 logSize = 0;
	glGetShaderiv(vShader, GL_INFO_LOG_LENGTH, &logSize);
	ansichar * shaderLog = reinterpret_cast<ansichar*>(gMalloc->malloc(logSize));
	glGetShaderInfoLog(vShader, logSize, &logSize, shaderLog);
	printf("%s\n", shaderLog); */

	glAttachShader(prog, vShader),
	glAttachShader(prog, fShader);
	glLinkProgram(prog);

	int32 compileStatus = 0;
	glGetShaderiv(vShader, GL_COMPILE_STATUS, &compileStatus);
	if (!compileStatus) printf("Compile error in vertex shader\n");
	glGetShaderiv(fShader, GL_COMPILE_STATUS, &compileStatus);
	if (!compileStatus) printf("Compile error in fragment shader\n");

	// Check link status
	int32 linkStatus = 0;
	glGetProgramiv(prog, GL_LINK_STATUS, &linkStatus);
	if (!linkStatus) printf("Link error\n");

	gMalloc->free(vShaderSource);
	gMalloc->free(fShaderSource);

	return prog;
}