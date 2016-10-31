# NRobot top level CMakeLists ###################################################
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET( CMAKE_VERBOSE_MAKEFILE off )

# Project name and version
PROJECT(NRobot)
SET(NR_VERSION_MAJOR 0)
SET(NR_VERSION_MINOR 0)

# Compile time parameters


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
    INCLUDE_DIRECTORIES(${SDL2_INCLUDE_DIRS})
ENDIF(SDL2_FOUND)

# Write cmake settings to header file
CONFIGURE_FILE(
"${PROJECT_SOURCE_DIR}/src/NRobot/NRConfig.hpp.in"
"${PROJECT_BINARY_DIR}/NRConfig.hpp"
)
INCLUDE_DIRECTORIES("${PROJECT_BINARY_DIR}")
INSTALL(FILES "${PROJECT_BINARY_DIR}/NRConfig.hpp" DESTINATION include/NRobot)

# Call cmake in subdirectories
ADD_SUBDIRECTORY(src)
