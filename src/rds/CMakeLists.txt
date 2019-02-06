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
set_target_properties(${PROJECT_NAME}

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

## Create executable
add_executable(ai_${PROJECT_NAME}

	./ai_main.cpp
	${SOURCES}
	${HEADERS}
)
set_target_properties(ai_${PROJECT_NAME}

	PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}
)

# Manage dependencies
find_package(FBXSDK REQUIRED)
target_link_libraries(ai_${PROJECT_NAME}

	sgl
	${FBXSDK_LIBRARIES}
	${CMAKE_DL_LIBS}
)
target_include_directories(ai_${PROJECT_NAME}

	PUBLIC
		./public
		${FBXSDK_INCLUDE_DIR}
)

# Enable run target
add_custom_target(ai

	COMMAND				${OUTPUT_DIR}/ai_${PROJECT_NAME}
	DEPENDS				ai_${PROJECT_NAME}
	WORKING_DIRECTORY	${PROJECT_SOURCE_DIR}
)