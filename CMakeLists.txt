# NRobot top level CMakeLists ##################################################
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET( CMAKE_VERBOSE_MAKEFILE off )

# Project name, version and language
PROJECT(NRobot VERSION 0.6.2 LANGUAGES CXX)

# Compile time parameters
# Time execution
SET(NR_TIMEIT 1)


############################# DO NOT EDIT THESE ################################
# Compiler flags
SET(CMAKE_CXX_STANDARD 11)
IF(CMAKE_CXX_COMPILER_ID STREQUAL GNU)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ansi -pedantic")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall -Wno-unused-function -Wno-unused-parameter -Wextra -Wpedantic -Wfatal-errors")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -w -O2")
ENDIF(CMAKE_CXX_COMPILER_ID STREQUAL GNU)

# Debug postfix
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    SET(BUILD_POSTFIX "_d")
else(CMAKE_BUILD_TYPE STREQUAL Debug)
    SET(BUILD_POSTFIX "")
endif(CMAKE_BUILD_TYPE STREQUAL Debug)

# Find SDL
FIND_PACKAGE(SDL2)
# SET(SDL2_FOUND false) # Uncomment to compile without SDL
IF(SDL2_FOUND)
    SET(NR_USE_SDL 1)
    INCLUDE_DIRECTORIES(${SDL2_INCLUDE_DIRS})
ELSE(SDL2_FOUND)
    SET(NR_USE_SDL 0)
ENDIF(SDL2_FOUND)

# Detect if the OS is Windows
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(NR_USING_WINDOWS 1)
ELSE(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(NR_USING_WINDOWS 0)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

# Write cmake settings to header file
CONFIGURE_FILE(
"${PROJECT_SOURCE_DIR}/src/N_Config.hpp.in"
"${PROJECT_BINARY_DIR}/src/N_Config.hpp"
)
INCLUDE_DIRECTORIES("${PROJECT_BINARY_DIR}/src")

# Call cmake in subdirectories
INCLUDE_DIRECTORIES(src)
ADD_SUBDIRECTORY(src)
