# Setup library ---------------------------------
## Gather source and headers files
file(GLOB_RECURSE SOURCES private

	*.c
	*.cpp
	*.cxx
)
file(GLOB_RECURSE HEADERS public

	*.h
	*.hpp
)

## Create library
add_library(${PROJECT_NAME}

	${SOURCES}
	${HEADERS}
)

## Include directories
target_include_directories(${PROJECT_NAME}

	PUBLIC
		public
)