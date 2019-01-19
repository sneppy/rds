#include "coremin.h"
#include "gldrv/gldrv.h"
#include "primitives/box.h"
#include <SDL.h>

Malloc * gMalloc = nullptr;

struct Hit
{
	vec3 hitPoint;
};

struct Cube
{
public:
	// Cube transform
	vec3 location;
	quat rotation;

	// Cube shape
	Box shape;

	// Physics
	float32 friction;
	float32 mass;
	vec3 speed;
	vec3 angularSpeed;

public:
	/// Default constructor
	Cube() : shape(vec3(-1.f), vec3(2.f)), friction(1.f), mass(1.5f), speed(0.f), angularSpeed(0.f) {};

	/// Apply impulsive force
	FORCE_INLINE void applyImpulse(const vec3 & d, const vec3 & p, float32 dt)
	{
		const vec3 com = (shape.min + shape.max) / 2.f;

		// Calc torque
		const vec3 torque = (rotation * (p - com)) ^ d;

		// Update speed
		speed += d / mass * dt;

		// Update angular speed
		const vec3 extent = shape.max - shape.min;
		const vec3 angularAcceleration = torque / (mass * (extent.x * extent.x) * (2.f / 5.f));
		angularSpeed += angularAcceleration * dt;

		(quat(angularSpeed.getSize(), angularSpeed.getNormal())).print();
	}

	/// Simualte physics
	FORCE_INLINE void tickPhysics(float32 dt)
	{
		// Damp speed
		speed -= (speed * friction * dt);
		angularSpeed -= (angularSpeed * friction * dt);

		// Update location and rotation
		location += speed * dt;
		if (!angularSpeed.isNearlyZero())
			rotation = quat(angularSpeed.getSize() * dt, angularSpeed.getNormal()) * rotation;
	}
};

ansichar * readFile(const String & filename);
uint32 createProgram();
bool hitUnderMouse(uint32 x, uint32 y, Hit & hit, const Array<Cube*> & cubes);

vec3 cameraLocation;
vec3 cameraSpeed;
quat cameraRotation;
mat4 viewProjectionMatrix;

const float32
	accelerationFactor = 64.f,
	brakeFactor = 4.f;

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

	/// Setup a nice little cube

	const uint32 numCubes = 256;
	Array<Cube*> cubes;
	for (uint32 i = 0; i < numCubes; ++i)
	{
		Cube * cube = new Cube;
		cube->location = vec3(Math::randf() - 0.5f, Math::randf() - 0.5f, Math::randf()) * 20.f;
		cube->rotation = quat(Math::randf() * M_PI_2, vec3(Math::randf() - 0.5f));
		cube->mass = Math::randf() + 0.5f;

		cubes += cube;
	}

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
	vertices.push((const Vertex[]){
		// Plane
		Vertex(-10.f, -2.f, -10.f),
		Vertex(-10.f, -2.f, 10.f),
		Vertex(10.f, -2.f, -10.f),
		Vertex(10.f, -2.f, 10.f),
	}, 4);

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
		dim3(5, 4, 0),
		dim3(8, 10, 9),
		dim3(10, 11, 9)
	}, 14);

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

	auto bu = *triangles;

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
	cameraLocation = vec3(0.f, 0.f, -3.f);

	while (!bShouldQuit)
	{
		// Update delta time
		currTick = SDL_GetPerformanceCounter();
		dt = (float32)(currTick - lastTick) / SDL_GetPerformanceFrequency();
		lastTick = currTick;

		printf("FPS: %u\n", (uint32)(1.f / dt));
		
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
				case SDL_MOUSEBUTTONDOWN:
					Hit hit;
					if (hitUnderMouse(
						event.button.x,
						event.button.y,
						hit,
						cubes
					))
						hit.hitPoint.print();
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
		cameraSpeed += (cameraAcceleration * accelerationFactor - cameraSpeed * brakeFactor) * dt;
		cameraLocation += cameraSpeed * dt;

		// Simulate physics
		for (const auto cube : cubes) cube->tickPhysics(dt);

		// Draw
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		viewProjectionMatrix = mat4::glProjection(M_PI_2, 0.5f) * mat4::rotation(!cameraRotation) * mat4::translation(-cameraLocation);
		glUniformMatrix4fv(uniforms["viewProjectionMatrix"], 1, GL_TRUE, viewProjectionMatrix.array);

		// Draw cubes
		for (const auto cube : cubes)
		{
			glUniformMatrix4fv(uniforms["modelMatrix"], 1, GL_TRUE, mat4::transform(cube->location, cube->rotation, vec3(1.f)).array);
			glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_INT, 0);
		}

		SDL_GL_SwapWindow(window);
	}
}

bool hitUnderMouse(uint32 x, uint32 y, Hit & hit, const Array<Cube*> & cubes)
{
	// Get ray from mouse
	const float32
		u = 2.f * x / 1920.f - 1.f,
		v = 2.f * y / 1080.f - 1.f;

	vec3 start(u, -v, -1.f), end;
	
	// Unproject ray
	start	= !viewProjectionMatrix * (vec4(start, 1.f) * 0.5f);
	end		= start + (start - cameraLocation) * 1000.f;
	(start - cameraLocation).print();

	for (const auto & cube : cubes)
	{
		// Start by removing those cubes we can't see
		if (((cube->location - cameraLocation).normalize() & cameraRotation.forward()) < 0.f)
			continue;
		
		const mat4 inverseModelMatrix = !mat4::transform(cube->location, cube->rotation);
		
		const vec3
			localStart	= inverseModelMatrix * start,
			localEnd	= inverseModelMatrix * end;

		vec3 localHit;
		if (cube->shape.intersect(localStart, localEnd, localHit))
		{
			printf("hit!\n");
			cube->applyImpulse((end - start).normalize() * 1000.f, localHit, 1.f / 60.f);
		}
	}

	return false;
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

	int32 compileStatus = 0;
	glGetShaderiv(vShader, GL_COMPILE_STATUS, &compileStatus);
	if (!compileStatus) printf("Compile status");
	glGetShaderiv(fShader, GL_COMPILE_STATUS, &compileStatus);
	if (!compileStatus) printf("Compile status");

	glAttachShader(prog, vShader),
	glAttachShader(prog, fShader);
	glLinkProgram(prog);

	// Check link status
	int32 linkStatus = 0;
	glGetProgramiv(prog, GL_LINK_STATUS, &linkStatus);
	if (!linkStatus) printf("Link error\n");

	gMalloc->free(vShaderSource);
	gMalloc->free(fShaderSource);

	return prog;
}