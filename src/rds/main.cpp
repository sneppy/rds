#include "coremin.h"
#include "gldrv/gldrv.h"
#include "primitives/box.h"
#include "fbx/importer.h"
#include "renderer/render_batch.h"
#include <SDL.h>

Malloc * gMalloc = nullptr;

ansichar * readFile(const String & filename);
uint32 createProgram();

vec3 cameraLocation;
vec3 cameraSpeed;
quat cameraRotation;
mat4 viewProjectionMatrix;

// Time variables
uint64 currTick;
uint64 lastTick;
float32 dt;

const float32
	accelerationFactor = 16.f,
	brakeFactor = 4.f;

class Wheel
{
public:
	/// Wheel relative transform
	vec3 offset;
	quat rotation;
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
		radius(0.31f),
		mass(20.f),
		inertiaMoment(mass * radius * radius / 2.f),
		staticCoeff(1.f),
		kineticCoeff(0.9f),
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
	vec3 centerOfRotation;

public:
	/// Default constructor
	Vehicle() :
		location(0.f),
		rotation(0.f, vec3::up),
		wheels(4),
		mass(1455.f),
		inertiaMoment((1.f / 12.f) * mass * (30.f)),
		centerOfMass(0.f, 0.31f, -0.2f),
		linearMomentum(0.f),
		linearVelocity(0.f),
		angularMomentum(0.f),
		angularVelocity(0.f),
		wheelsForce(0.f),
		wheelsTorque(0.f),
		centerOfRotation(0.f) {}
	
	/// Setup wheels
	FORCE_INLINE void setupWheels()
	{
		wheels(0) = Wheel(vec3(-1.587 / 2.f, 0.31f, 1.41f)); // Left
		wheels(1) = Wheel(vec3(1.587 / 2.f, 0.31f, 1.41f));
		wheels(2) = Wheel(vec3(-1.612 / 2.f, 0.31f, -1.28f));
		wheels(3) = Wheel(vec3(1.612 / 2.f, 0.31f, -1.28f)); 

		wheels[0].vehicle = this;
		wheels[1].vehicle = this;
		wheels[2].vehicle = this;
		wheels[3].vehicle = this;

		wheels[0].orientation = quat(M_PI, vec3::up),
		wheels[2].orientation = quat(M_PI, vec3::up);
	}

	/// Set throttle
	FORCE_INLINE void setThrottle(float32 value)
	{
		wheels[0].throttle = value;
		wheels[1].throttle = value;
		/* wheels[2].throttle = value;
		wheels[3].throttle = value; */
	}

	/// Set steering on wheels
	FORCE_INLINE void setSteering(float32 value)
	{
		const float32
			axleRadius = 0.2f,
			axleMaxLength = 1.2f,
			axleMinLength = 1.f;

		// Ok, this is nice, but a simpler and
		// better performing way is to force
		// the angles to intersect in the instant
		// center of rotation
		
		const float32
			steeringAngle = value * 0.4f * (Math::pow(2.71, -linearVelocity.getSquaredSize() / 400.f)),
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
		
		wheels[2].steeringAngle = 0.f,
		wheels[3].steeringAngle = 0.f;
	}

	/// Simulate physics
	FORCE_INLINE void tickPhysics(float32 dt)
	{
		// Delegate to wheels
		for (auto & wheel : wheels)
			wheel.tickPhysics(dt);

		//wheelsForce.print();

		// Update momentum
		linearMomentum	+= (rotation * wheelsForce) * dt;
		angularMomentum	+= wheelsTorque * dt;

		linearVelocity	= linearMomentum / mass;
		angularVelocity	= angularMomentum / inertiaMoment;

		//printf("Cruise velocity: %.1f km/h\n", linearVelocity.getSize() * 3.6f);

		// Update instant center of rotation
		// Center of rotation should be the
		// same, computed with the left and
		// right wheels. Right now it's not
		const float32 wheelsDistance = (wheels[0].offset - wheels[2].offset).getSize();
		if (Math::abs(wheels[0].steeringAngle) > FLT_EPSILON)
			centerOfRotation = (wheelsDistance / Math::sin(wheels[0].steeringAngle)) * quat(wheels[0].steeringAngle, vec3::up).right();
		else
			centerOfRotation = 0.f;

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
			transforms += vehicleTransform * mat4::transform(wheel.offset, wheel.rotation);
		
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
	const vec3 tractiveForce = steeringOrientation.forward() * 2400.f * throttle;

	// Rolling resistance
	const float32 wheelDeformation = 0.01;
	const vec3 rollingResistance = -(!vehicle->rotation * vehicle->linearVelocity.getSafeNormal()) * (suspensionForce.getSize() * wheelDeformation) / Math::sqrt(radius * radius - wheelDeformation * wheelDeformation);

	// Distributed drag resistance
	const vec3 dragResistance = -(!vehicle->rotation * vehicle->linearVelocity) * vehicle->linearVelocity.getSize() * 0.39f / 4.f;

	// Forward force
	vec3 forwardForce = tractiveForce + rollingResistance + dragResistance;

	if (throttle < 0.f)
	{
		float32 brakeForce = 9600.f * throttle;
		forwardForce += Math::min(staticFriction, brakeForce) * steeringOrientation.forward();
	}
	
	// Lateral force
	const vec3 centerOffset = (offset - vehicle->centerOfMass);
	const vec3 vehicleForwardMomentum = !vehicle->rotation * (vehicle->linearMomentum / 4.f);
	const vec3 vehicleTangentialMomentum = ((vehicle->angularMomentum / 4.f) ^ centerOffset) / centerOffset.getSquaredSize();
	const vec3 lateralForce = ((vehicleForwardMomentum + vehicleTangentialMomentum) & steeringOrientation.right()) * steeringOrientation.left();

	// Check friction
	vec3 groundForce = lateralForce / dt + forwardForce;
	if (groundForce.getSize() > staticFriction)
	{
		printf("too much! %.2fG / %.2fG\n", groundForce.getSize() / (vehicle->mass / 4.f * 9.81f), staticFriction / (vehicle->mass / 4.f * 9.81f));
		groundForce *= kineticFriction / groundForce.getSize();
	}
	
	// Apply force to chassis
	vehicle->applyForce(groundForce, offset);

	// Update wheel rotation
	angularVelocity = vec3::right * vehicle->linearVelocity.getSize() / radius;
	if (!angularVelocity.isNearlyZero())
		phase = quat(angularVelocity.getSize() * dt, angularVelocity.getNormal()) * phase;

	// Show wheel orientation
	rotation = steeringOrientation * phase * orientation;
}

int main()
{
	Memory::createGMalloc();
	srand(clock());

	// Setup input
	Map<uint32, int32> keys;
	Map<uint32, float32> axes;
	point2 cursorLocation, cursorDelta;

	// Setup car
	Vehicle car;
	car.setupWheels();

	// Import meshes
	MeshData chassisMesh;
	MeshData wheelMesh;

	Importer importer;

	importer.loadScene("assets/vehicle/GL_Shelby_ChassisOnlyY.fbx");
	importer.importStaticMesh(chassisMesh);

	importer.loadScene("assets/vehicle/GL_Shelby_WheelOnly.fbx");
	importer.importStaticMesh(wheelMesh);

	// Init keys
	keys[SDLK_w] = 0,
	keys[SDLK_a] = 0,
	keys[SDLK_s] = 0,
	keys[SDLK_d] = 0,
	keys[SDLK_SPACE] = 0,
	keys[SDLK_LCTRL] = 0;
	axes[4] = -1.f;
	axes[5] = -1.f;

	bool bShouldQuit = false;

	initOpenGL();

	SDL_Window * window = SDL_CreateWindow("rds", 0, 0, 1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_BORDERLESS);
	SDL_GLContext context = SDL_GL_CreateContext(window);
	SDL_GL_SetSwapInterval(-1);

	// Context requires a vao
	uint32 vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	SDL_GameController * gamepad = nullptr;
	for (uint8 i = 0; i < SDL_NumJoysticks(); ++i)
	{
		if (SDL_IsGameController(i))
		{
			printf("found controller %s\n", SDL_GameControllerNameForIndex(i));
			gamepad = SDL_GameControllerOpen(i);
		}
	}

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

	uint32 prog = createProgram();
	glUseProgram(prog);

	// Setup vehicle render batch
	RenderBatch vehicleBatch;
	vehicleBatch.addMesh(&chassisMesh, "Chassis");
	vehicleBatch.upload();

	RenderBatch wheelBatch;
	wheelBatch.addMesh(&wheelMesh, "Wheel");
	wheelBatch.upload();

	// Setup cube in opengl
	uint32 vbo, ebo;
	glGenBuffers(1, &vbo);
	glGenBuffers(1, &ebo);

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
	currTick = SDL_GetPerformanceCounter();
	lastTick = currTick;
	dt = 0.f;

	// Positions of reference cubes
	Array<mat4> cubes(128);
	for (uint8 i = 0; i < cubes.getSize(); ++i)
		cubes += mat4::translation(vec3(Math::randf(), 0.f, Math::randf()) * 100.f);

	// Camera setup
	cameraLocation = vec3(0.f, 1.f, -5.f);
	cameraRotation = quat(M_PI_2, vec3::right);

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
				case SDL_JOYAXISMOTION:
					axes[(uint32)event.caxis.axis] = event.caxis.value / 32768.f;
					break;
			}
		}

		// Handle input
		const vec3 cameraDirection(cameraRotation.forward());
		const float32
			movementX = keys[SDLK_d] - keys[SDLK_a],
			movementZ = keys[SDLK_w] - keys[SDLK_s],
			movementY = keys[SDLK_SPACE] - keys[SDLK_LSHIFT];

		//cameraRotation = quat((movementX - axes[2]) * M_PI * dt, vec3::up) * cameraRotation;
		
		// Camera location
		vec3 cameraAcceleration(movementX, movementY, movementZ);
		cameraSpeed += ((/* cameraRotation *  */cameraAcceleration) * accelerationFactor - cameraSpeed * brakeFactor) * dt;
		cameraLocation += cameraSpeed * dt;

		//cameraLocation = Math::lerp(cameraLocation, car.location + (((quat)(cameraRotation * car.rotation)).backward() + vec3::up * 0.3f) * 10.f, 0.2f);
		const vec3 targetCameraLocation = car.location + car.rotation.up() * 25.f;
		cameraLocation = Math::lerp(cameraLocation, targetCameraLocation, 0.25f);

		car.setThrottle((float32)keys[SDLK_UP] + (axes[4] + 1.f) / 2.f);
		car.setSteering((float32)(keys[SDLK_RIGHT] - keys[SDLK_LEFT]) + axes[0]);

		// Simulate physics
		car.tickPhysics(dt);

		// Draw
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		viewProjectionMatrix = mat4::glProjection(M_PI_2, 0.5f) * mat4::rotation(/* !(quat)(cameraRotation * car.rotation) */!cameraRotation) * mat4::translation(-cameraLocation);
		glUniformMatrix4fv(uniforms["viewProjectionMatrix"], 1, GL_TRUE, viewProjectionMatrix.array);

		// Draw car
		glBindBuffer(GL_ARRAY_BUFFER, vbo),
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		/// Reference cubes
		for (const auto & cube : cubes)
		{
			glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, cube.array);
			glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_INT, 0);
		}

		// Draw chassis
		vehicleBatch.bind();
		glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, mat4::transform(car.location, car.rotation).array);
		vehicleBatch.draw();

		// Draw wheels
		wheelBatch.bind();
		for (const auto wheelTransform : car.getWheelsTransforms())
		{
			glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, wheelTransform.array);
			wheelBatch.draw();
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