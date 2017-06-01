# NPart top level CMakeLists ###################################################
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET( CMAKE_VERBOSE_MAKEFILE off )

# Project name and version
PROJECT(NPart)
SET(NP_VERSION_MAJOR 0)
SET(NP_VERSION_MINOR 1)

# Compile time parameters
# Whether to use 3D or 2D points
SET(NP_3D_POINTS true)
# Error tolerance when comparing points
SET(NP_CMP_ERR 0.00000001)
# Floating point precision
SET(NPFLOAT double)
# Default number of points per circle
SET(NP_PPC 120)

# Compiler flags
SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wno-unused-function")
SET(CMAKE_CXX_FLAGS_RELEASE "-w -O2")



############################# DO NOT EDIT THESE ################################

# Debug postfix
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    SET(BUILD_POSTFIX "_d")
else(CMAKE_BUILD_TYPE STREQUAL Debug)
    SET(BUILD_POSTFIX "")
endif(CMAKE_BUILD_TYPE STREQUAL Debug)

# Find SDL
# https://groups.google.com/forum/#!topic/linux.debian.bugs.dist/j9WDz2YHeKg
# There seems to be a bug in sdl2-config.cmake, use the one provided
FIND_PACKAGE(SDL2)
IF(SDL2_FOUND)
    SET(NP_USE_SDL true)
    INCLUDE_DIRECTORIES(${SDL2_INCLUDE_DIRS})
ELSE(SDL2_FOUND)
    SET(NP_USE_SDL false)
ENDIF(SDL2_FOUND)

# Write cmake settings to header file
CONFIGURE_FILE(
"${PROJECT_SOURCE_DIR}/src/NPart/NPConfig.hpp.in"
"${PROJECT_BINARY_DIR}/NPConfig.hpp"
)
INCLUDE_DIRECTORIES("${PROJECT_BINARY_DIR}")
INSTALL(FILES "${PROJECT_BINARY_DIR}/NPConfig.hpp" DESTINATION include/NPart)

# Call cmake in subdirectories
ADD_SUBDIRECTORY(src)
