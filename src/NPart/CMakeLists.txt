# NPart CMakeLists ###########################################################

# Source files
SET(N_NPART_SRC "clipper.cpp" "NClip.cpp" "NPart.cpp")

# Create library
ADD_LIBRARY(npart STATIC ${N_NPART_SRC})

# Set library postfix
SET_TARGET_PROPERTIES(npart PROPERTIES OUTPUT_NAME npart${BUILD_POSTFIX})
