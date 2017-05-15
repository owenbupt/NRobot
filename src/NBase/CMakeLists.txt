# GOL_SDL CMakeLists ###########################################################

# Source files
SET(NR_NBASE_SRC "NBase.cpp")

# Create library
ADD_LIBRARY(nbase STATIC ${NR_NBASE_SRC})

# Set library postfix
SET_TARGET_PROPERTIES(nbase PROPERTIES OUTPUT_NAME nbase${BUILD_POSTFIX})
