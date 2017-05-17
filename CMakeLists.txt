# NRobot top level CMakeLists ##################################################
CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
SET( CMAKE_VERBOSE_MAKEFILE off )

# Project name, version and language
PROJECT(NRobot VERSION 0.3.0 LANGUAGES CXX)

##### Compile time parameters #####
# Compile with plot functionality using SDL 2
SET(NR_PLOT_ENABLE 1)
# Time execution
SET(NR_TIME_EXECUTION 0)


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
IF(NR_PLOT_ENABLE)
    FIND_PACKAGE(SDL2)
    IF(SDL2_FOUND)
        SET(NR_PLOT_AVAILABLE 1)
        INCLUDE_DIRECTORIES(${SDL2_INCLUDE_DIRS})
    ELSE(SDL2_FOUND)
        SET(NR_PLOT_AVAILABLE 0)
    ENDIF(SDL2_FOUND)
ENDIF(NR_PLOT_ENABLE)

# Write cmake settings to header file
CONFIGURE_FILE(
"${PROJECT_SOURCE_DIR}/src/NRBase/NRConfig.hpp.in"
"${PROJECT_BINARY_DIR}/src/NRBase/NRConfig.hpp"
)
# Include generated header file
INCLUDE_DIRECTORIES("${PROJECT_BINARY_DIR}/src/NRBase")

# Call cmake in subdirectories
INCLUDE_DIRECTORIES(src)
ADD_SUBDIRECTORY(src)
