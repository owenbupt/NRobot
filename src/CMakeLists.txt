# NRobot src CMakeLists ########################################################

########## Create nrobot library ##########
# Include headers
INCLUDE_DIRECTORIES(NRBase)
INCLUDE_DIRECTORIES(NRPart)
INCLUDE_DIRECTORIES(NRPlot)
INCLUDE_DIRECTORIES(NRobot)

# Source files to compile
SET(NRBASE_SRC "NRBase/clipper.cpp" "NRBase/NRBase.cpp")
SET(NRPART_SRC "NRPart/NRPart.cpp")
SET(NROBOT_SRC "NRobot/NRobot.cpp" "NRobot/Jn.cpp" "NRobot/n.cpp" "NRobot/J.cpp")
IF(NR_PLOT_AVAILABLE)
    SET(NRPLOT_SRC "NRPlot/NRPlot.cpp")
ENDIF(NR_PLOT_AVAILABLE)

# Create static library
ADD_LIBRARY(nrobot STATIC ${NRBASE_SRC} ${NRPART_SRC} ${NROBOT_SRC} ${NRPLOT_SRC})
SET_TARGET_PROPERTIES(nrobot PROPERTIES OUTPUT_NAME nrobot${BUILD_POSTFIX})

# Install library
# Install headers inside NRobot folder
# Install static library in standard folder



########## Create simulation executables ##########
SET(SIM_SRC "sim/sim_ground.cpp" "sim/sim_ground_anisotropic.cpp" "sim/sim_ground_anisotropic_switching.cpp" "sim/sim_ground_distance.cpp" "sim/sim_ground_antagonistic.cpp")
FOREACH( sim ${SIM_SRC} )
    # Get base filename
    GET_FILENAME_COMPONENT( sim_name ${sim} NAME_WE )
    ADD_EXECUTABLE(SIM_${sim_name} ${sim})
    TARGET_LINK_LIBRARIES(SIM_${sim_name} nrobot ${SDL2_LIBRARIES})
    SET_TARGET_PROPERTIES(SIM_${sim_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    SET_TARGET_PROPERTIES(SIM_${sim_name} PROPERTIES OUTPUT_NAME ${sim_name}${BUILD_POSTFIX})
    MESSAGE( ${sim} ${sim_name} )
ENDFOREACH( sim ${SIM_SRC} )

########## Create testing executables ##########
SET(TEST_SRC "tests/test_NRBase.cpp" "tests/test_NRPlot.cpp" "tests/test_NRPart.cpp" "tests/test_AWGV_jacobians_normals_new.cpp" "tests/test_AU_partitioning.cpp")
FOREACH( test ${TEST_SRC} )
    # Get base filename
    GET_FILENAME_COMPONENT( test_name ${test} NAME_WE )
    ADD_EXECUTABLE(TEST_${test_name} ${test})
    TARGET_LINK_LIBRARIES(TEST_${test_name} nrobot ${SDL2_LIBRARIES})
    SET_TARGET_PROPERTIES(TEST_${test_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    SET_TARGET_PROPERTIES(TEST_${test_name} PROPERTIES OUTPUT_NAME ${test_name}${BUILD_POSTFIX})
    MESSAGE( ${test} ${test_name} )
ENDFOREACH( test ${TEST_SRC} )
