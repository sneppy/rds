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
	/// World space location and rotation
	vec3 location;
	quat rotation;

	/// Physics state
	vec3 linearMomentum;
	vec3 angularMomentum;

	/// Current torque applied to the wheel
	vec3 torque;

	/// Current steering angle and wheel phase
	quat wheelPhase;
	quat steeringAngle;

	/// Wheel geometric and physics properties
	float32 inertialMass;
	float32 radius;
	float32 pressure;

public:
	/// Default constructor
	Wheel() :
		location(0.f),
		rotation(0.f, vec3::up),
		linearMomentum(0.f),
		angularMomentum(0.f),
		torque(0.f),
		inertialMass(20.f),
		radius(0.24f),
		pressure(2.1f) {}

	/// Apply motor force to wheel
	FORCE_INLINE void applyMotorForce(float32 f)
	{
		/// Motor force always spins in forward direction
		torque += vec3::right * f * 20.f;
	}

	/// Apply brake force to wheel
	FORCE_INLINE void applyBrakeForce(float32 f)
	{
		/// Brake force stops the wheel rotation
		if (!angularMomentum.isNearlyZero())
			torque += -angularMomentum.getNormal() * f * 100.f;
	}

	/// Simulate physics
	FORCE_INLINE void tickPhysics(float32 dt)
	{
		/// @see https://www.engineeringtoolbox.com/rolling-friction-resistance-d_1303.html
		/// Apply rolling friction
		const float32 linearSpeedMagnitude = (linearMomentum / inertialMass).getSize();
		const float32 frictionCoeff = 0.005f + (1.f / pressure) * (0.01f + (linearSpeedMagnitude * linearSpeedMagnitude / 771.6f));

		/// Apply rolling friction
		const float32 normalForce = inertialMass * 9.81f;
		if (!angularMomentum.isNearlyZero())
			torque += -angularMomentum.getNormal() * frictionCoeff * radius * normalForce;	

		/// Update momentum
		angularMomentum += torque * dt;
		linearMomentum = (angularMomentum ^ vec3::up) / radius;

		angularMomentum.print();

		/// Update location and rotation
		const float32 inertialMoment = inertialMass * radius * radius;
		if (!angularMomentum.isNearlyZero())
			wheelPhase = quat((angularMomentum.getSize() / inertialMoment) * dt, angularMomentum.getNormal()) * wheelPhase;
		
		rotation = steeringAngle * wheelPhase;
		location += (steeringAngle * linearMomentum / inertialMass) * dt;

		/// Consume torque
		torque = 0.f;
	}
};

int main()
{
	Memory::createGMalloc();
	srand(clock());

	// Setup input
	Map<uint32, int32> keys;
	point2 cursorLocation, cursorDelta;

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

	// Setup wheel
	Wheel lWheel, rWheel;
	lWheel.location = vec3(-1.f, 0.f, 0.f);
	rWheel.location = vec3(1.f, 0.f, 0.f);

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

	// Camera setup
	cameraLocation = vec3(0.f, 3.f, 1.f);
	cameraRotation = quat(M_PI_4, vec3::right);

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

		const float32
			r = 1.f,
			l = 6.f,
			L = 7.f,
			alpha = M_PI / 3.f;
		
		float32
			steeringAngle = (keys[SDLK_RIGHT] - keys[SDLK_LEFT]) / 2.5f,
			a = Math::abs(steeringAngle) + alpha,
			A = Math::sin(a),
			B = (L / r) - Math::cos(a),
			C = 1.f - (L / r) * Math::cos(a) + (L * L - l * l) / (2.f * r * r),
			resultingAngle = 2.f * Math::atan((A + Math::sqrt(A * A + B * B - C * C)) / (B + C)),
			lWheelAngle = steeringAngle > 0.f ? steeringAngle : resultingAngle - alpha,
			rWheelAngle = steeringAngle > 0.f ? alpha - resultingAngle : steeringAngle;

		//printf("l: %f, r: %f\n",lWheelAngle, rWheelAngle);

		lWheel.steeringAngle = quat(lWheelAngle, vec3::up);
		rWheel.steeringAngle = quat(rWheelAngle, vec3::up);

		lWheel.applyMotorForce(keys[SDLK_UP]);
		lWheel.applyBrakeForce(keys[SDLK_DOWN]);

		rWheel.applyMotorForce(keys[SDLK_UP]);
		rWheel.applyBrakeForce(keys[SDLK_DOWN]);
		
		// Camera location
		vec3 cameraAcceleration(movementX, movementY, movementZ);
		cameraSpeed += ((cameraRotation * cameraAcceleration) * accelerationFactor - cameraSpeed * brakeFactor) * dt;
		cameraLocation += cameraSpeed * dt;

		// Simulate physics
		lWheel.tickPhysics(dt);
		rWheel.tickPhysics(dt);

		// Draw
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		viewProjectionMatrix = mat4::glProjection(M_PI_2, 0.5f) * mat4::rotation(!cameraRotation) * mat4::translation(-cameraLocation);
		glUniformMatrix4fv(uniforms["viewProjectionMatrix"], 1, GL_TRUE, viewProjectionMatrix.array);

		// Draw cubes
		glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, mat4::transform(lWheel.location, lWheel.rotation, vec3(0.2f)).array);
		glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_INT, 0);

		glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, mat4::transform(rWheel.location, rWheel.rotation, vec3(0.2f)).array);
		glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_INT, 0);

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