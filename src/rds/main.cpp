#include "coremin.h"
#include "gldrv/gldrv.h"
#include "primitives/box.h"
#include "fbx/importer.h"
#include "renderer/render_batch.h"
#include "physics/vehicle.h"
#include "physics/vehicle-wheel.h"
#include <SDL.h>

Malloc * gMalloc = nullptr;

ansichar * readFile(const String & filename);
uint32 createProgram();

vec3 cameraLocation;
vec3 cameraSpeed;
quat cameraRotation;
mat4 viewProjectionMatrix;

/// @todo delete
float32 carThrottle = 0.f;
float32 carBrake = 0.f;

// Time variables
uint64 currTick;
uint64 lastTick;
float32 dt;

const float32
	accelerationFactor = 16.f,
	brakeFactor = 4.f;

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
	axes[3] = -1.f;

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
		cubes += mat4::translation(vec3(Math::randf(), 1.f / 100.f, Math::randf()) * 100.f);

	// Camera setup
	cameraLocation = vec3(0.f, 1.f, -5.f);
	cameraRotation = quat(M_PI_2 / 2.f, vec3(1.f, 1.f, 0.f));

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
		const vec3 targetCameraLocation = car.location + cameraRotation.backward() * 10.f;
		cameraLocation = Math::lerp(cameraLocation, targetCameraLocation, 1.f);

		/* car.setThrottle((float32)keys[SDLK_UP] + (axes[4] + 1.f) / 2.f);
		car.setSteering((float32)(keys[SDLK_RIGHT] - keys[SDLK_LEFT]) + axes[0]); */
		{
			const float32
				axleRadius = 0.2f,
				axleMaxLength = 1.2f,
				axleMinLength = 1.f,
				value = (float32)(keys[SDLK_RIGHT] - keys[SDLK_LEFT]) + axes[0];

			// Ok, this is nice, but a simpler and
			// better performing way is to force
			// the angles to intersect in the instant
			// center of rotation
			
			const float32
				steeringAngle = value * 0.4f/*  * (Math::pow(1.5f, -car.chassis.linearVelocity.getSquaredSize() / 400.f)) */,
				alpha = M_PI / 3.f + Math::abs(steeringAngle),
				A = Math::sin(alpha),
				B = (axleMaxLength / axleRadius) - Math::cos(alpha),
				C = 1.f - (axleMaxLength / axleRadius) * Math::cos(alpha) + (axleMaxLength * axleMaxLength - axleMinLength * axleMinLength) / (2.f * axleRadius * axleRadius),
				resultAngle = 2.f * Math::atan((A + Math::sqrt(A * A + B * B - C * C)) / (B + C));
			
			const float32
				leftWheelAngle	= value > 0.f ? steeringAngle : -M_PI / 3.f + resultAngle,
				rightWheelAngle	= value > 0.f ? M_PI / 3.f - resultAngle : steeringAngle;
			
			car.wheels[0]->steeringAngle = Math::lerp(car.wheels[0]->steeringAngle, leftWheelAngle, 0.2f),
			car.wheels[1]->steeringAngle = Math::lerp(car.wheels[1]->steeringAngle, rightWheelAngle, 0.2f);
			
			car.wheels[2]->steeringAngle = 0.f,
			car.wheels[3]->steeringAngle = 0.f;
		}
		carThrottle	= (float32)keys[SDLK_UP] + (axes[4] + 1.f) / 2.f;
		carBrake	= (float32)keys[SDLK_DOWN] + (axes[3] + 1.f) / 2.f;

		// Simulate physics
		car.tick(dt);

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
		glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, car.localTransform.array);
		vehicleBatch.draw();

		// Draw wheels
		wheelBatch.bind();
		for (auto wheel : car.wheels)
		{
			glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, (car.localTransform * wheel->localTransform).array);
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