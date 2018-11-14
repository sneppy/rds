cmake_minimum_required(VERSION 3.12)

# Project setup ---------------------------------
project(rds)

# Compiler setup --------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx -ggdb -O1")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Setup code ------------------------------------
## Vendors
add_subdirectory(vendor)

## Source code
add_subdirectory(src)