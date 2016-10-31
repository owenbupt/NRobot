# NRobot CMakeLists #####################################################

# Source files
SET(NROBOT_SRC NRobot.cpp NRControl.cpp)

# Create library
ADD_LIBRARY(nrobot SHARED ${NROBOT_SRC})

# Set library postfix
SET_TARGET_PROPERTIES(nrobot PROPERTIES OUTPUT_NAME nrobot${BUILD_POSTFIX})

# Install library and headers
INSTALL(TARGETS nrobot LIBRARY DESTINATION lib)
INSTALL(DIRECTORY . DESTINATION include/NRobot FILES_MATCHING PATTERN "*.hpp")
