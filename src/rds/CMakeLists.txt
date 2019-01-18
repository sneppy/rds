# Setup application -----------------------------
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

## Create executable
add_executable(${PROJECT_NAME}

	./main.cpp
	${SOURCES}
	${HEADERS}
)
set(OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)
set_target_properties(rds

	PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}
)

# Manage dependencies
find_package(FBXSDK REQUIRED)
target_link_libraries(${PROJECT_NAME}

	sgl
	${FBXSDK_LIBRARIES}
	${CMAKE_DL_LIBS}
)
target_include_directories(${PROJECT_NAME}

	PUBLIC
		./public
		${FBXSDK_INCLUDE_DIR}
)

# Enable run target
add_custom_target(run

	COMMAND				${OUTPUT_DIR}/${PROJECT_NAME}
	DEPENDS				${PROJECT_NAME}
	WORKING_DIRECTORY	${PROJECT_SOURCE_DIR}
)