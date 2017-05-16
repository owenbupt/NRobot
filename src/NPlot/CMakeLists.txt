# GOL_SDL CMakeLists ###########################################################

# Source files
SET(NR_NBASE_SRC "NPlot.cpp")

# Create library
ADD_LIBRARY(nplot STATIC ${NR_NBASE_SRC})

# Set library postfix
SET_TARGET_PROPERTIES(nplot PROPERTIES OUTPUT_NAME nplot${BUILD_POSTFIX})
