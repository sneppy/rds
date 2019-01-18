# Setup library ---------------------------------
project(sgl)

## Gather source and headers files
file(GLOB_RECURSE SOURCES

	private/*.c
	private/*.cpp
	private/*.cxx
)
file(GLOB_RECURSE HEADERS public

	public/*.h
	public/*.hpp
	public/*.tpp
)

## Create library
add_library(${PROJECT_NAME}

	${SOURCES}
	${HEADERS}
)

## Manage dependencies
### OpenGL
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

if(NOT SDL2_FOUND)
	message(FATAL_ERROR "SDL2 not found! `sudo apt-get install libsdl2-dev`")
endif(NOT SDL2_FOUND)

if(NOT OpenGL_FOUND)
	message(FATAL_ERROR "No GL library found!")
endif(NOT OpenGL_FOUND)

## Link third-party libraries
target_link_libraries(${PROJECT_NAME}

	${SDL2_LIBRARIES}
	${OPENGL_LIBRARIES}
)

## Include directories
target_include_directories(${PROJECT_NAME}

	PUBLIC
		public
		${SDL2_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
)