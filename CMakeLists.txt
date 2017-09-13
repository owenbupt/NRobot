# NRobot top level CMakeLists ##################################################
CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
SET( CMAKE_VERBOSE_MAKEFILE off )

# Project name, version and language
PROJECT(NRobot VERSION 0.6.2 LANGUAGES CXX)

##### Compile time parameters #####
# Compile with plot functionality using SDL 2
SET(NR_PLOT_ENABLE 1)
# Time execution
SET(NR_TIME_EXECUTION 1)








############################# DO NOT EDIT THESE ################################
# Compiler flags
SET(CMAKE_CXX_STANDARD 11)
IF(CMAKE_CXX_COMPILER_ID STREQUAL GNU)
    SET(CMAKE_CXX_FLAGS_DEBUG "-g -Og -Wall -Wno-unused-function -Wno-unused-parameter -Wextra -Wpedantic -Wfatal-errors")
    SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -w")
ENDIF(CMAKE_CXX_COMPILER_ID STREQUAL GNU)

# Debug postfix
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    SET(BUILD_POSTFIX "_d")
else(CMAKE_BUILD_TYPE STREQUAL Debug)
    SET(BUILD_POSTFIX "")
endif(CMAKE_BUILD_TYPE STREQUAL Debug)

# Find SDL2 if plot functionality is desired
IF(NR_PLOT_ENABLE)
    FIND_PACKAGE(SDL2)
    IF(SDL2_FOUND)
        SET(NR_PLOT_AVAILABLE 1)
        INCLUDE_DIRECTORIES(${SDL2_INCLUDE_DIRS})
    ELSE(SDL2_FOUND)
        SET(NR_PLOT_AVAILABLE 0)
        MESSAGE(WARNING "SDL2 was not found. Plot functionality will not be available")
    ENDIF(SDL2_FOUND)
ELSE(NR_PLOT_ENABLE)
    SET(NR_PLOT_AVAILABLE 0)
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
