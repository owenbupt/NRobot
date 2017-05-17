# NBase CMakeLists ###########################################################

# Source files
SET(N_NBASE_SRC "NBase.cpp")

# Create library
ADD_LIBRARY(nbase STATIC ${N_NBASE_SRC})

# Set library postfix
SET_TARGET_PROPERTIES(nbase PROPERTIES OUTPUT_NAME nbase${BUILD_POSTFIX})
