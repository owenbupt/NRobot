# NPlot CMakeLists ###########################################################

# Source files
SET(N_NPLOT_SRC "NPlot.cpp")

# Create library
ADD_LIBRARY(nplot STATIC ${N_NPLOT_SRC})

# Set library postfix
SET_TARGET_PROPERTIES(nplot PROPERTIES OUTPUT_NAME nplot${BUILD_POSTFIX})
