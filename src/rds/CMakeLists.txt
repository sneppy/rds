# Executable setup ------------------------------
## Gather source and header files
file(GLOB_RECURSE SOURCES private

	*.c
	*.cpp
	*.cxx
)
file(GLOB_RECURSE HEADERS public

	*.h
	*.hpp
)

## Create executable
add_executable(${PROJECT_NAME}

	${SOURCES}
	${HEADERS}
)

## Link libraries
target_link_libraries(${PROJECT_NAME}

	sgl
)

## Include directories
target_include_directories(${PROJECT_NAME}

	PUBLIC
		public
)

## Setup target run
add_custom_target(run

	COMMAND				${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}
	WORKING_DIRECTORY	${CMAKE_SOURCE_DIR}
	DEPENDS				${PROJECT_NAME}
)