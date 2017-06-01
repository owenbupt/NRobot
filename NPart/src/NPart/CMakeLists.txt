# NPart CMakeLists #####################################################


#################### npart ####################
# Source files
SET(NPART_SRC NPart.cpp NPBase.cpp NPClip.cpp clipper.cpp)

# Create library
ADD_LIBRARY(npart SHARED ${NPART_SRC})

# Set library postfix
SET_TARGET_PROPERTIES(npart PROPERTIES OUTPUT_NAME npart${BUILD_POSTFIX})

# Install library and headers
INSTALL(TARGETS npart LIBRARY DESTINATION lib)
INSTALL(DIRECTORY . DESTINATION include/NPart FILES_MATCHING PATTERN "*.hpp")



#################### npsdl ####################
IF(SDL2_FOUND)
    # Source files
    SET(NPSDL_SRC NPSDL.cpp)

    # Create library
    ADD_LIBRARY(npsdl SHARED ${NPSDL_SRC})

    # Set library postfix
    SET_TARGET_PROPERTIES(npsdl PROPERTIES OUTPUT_NAME npsdl${BUILD_POSTFIX})

    # Install library and headers
    INSTALL(TARGETS npsdl LIBRARY DESTINATION lib)
    INSTALL(DIRECTORY . DESTINATION include/NPart FILES_MATCHING PATTERN "*.hpp")
ENDIF(SDL2_FOUND)
