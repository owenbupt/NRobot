# NRobot src CMakeLists #########################################################

# Create testing executables
ADD_EXECUTABLE(YSUQ_TEST TEST_YSUQ_law.cpp)

# NRobot
INCLUDE_DIRECTORIES(NRobot)
ADD_SUBDIRECTORY(NRobot)
LINK_DIRECTORIES(/usr/local/lib)

# Link the libraries
IF(SDL2_FOUND)
    TARGET_LINK_LIBRARIES(YSUQ_TEST npart npsdl nrobot ${SDL2_LIBRARIES})
ELSE(SDL2_FOUND)
    TARGET_LINK_LIBRARIES(YSUQ_TEST npart npsdl nrobot)
ENDIF(SDL2_FOUND)

# Set executable output directory and postfix
SET_TARGET_PROPERTIES(YSUQ_TEST PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
SET_TARGET_PROPERTIES(YSUQ_TEST PROPERTIES OUTPUT_NAME YSUQ_TEST${BUILD_POSTFIX})
