# NRobot src CMakeLists ########################################################

# Libraries to compile
IF(GOL_USE_SDL)
    INCLUDE_DIRECTORIES(GOL_SDL)
    ADD_SUBDIRECTORY(GOL_SDL)
    SET(GOL_SDL_LIB golsdl)
ENDIF(GOL_USE_SDL)

# Source files to compile
SET(GOL_SRC "GOL.cpp")

# Create executable
ADD_EXECUTABLE(GameOfLyfe "GOL_Main.cpp" ${GOL_SRC})

# Link the libraries
# SET(CMAKE_LINK_SEARCH_START_STATIC TRUE)
# SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
TARGET_LINK_LIBRARIES(GameOfLyfe ${SDL2_LIBRARIES} ${GOL_SDL_LIB})

# Set executable output directory and postfix
SET_TARGET_PROPERTIES(GameOfLyfe PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
SET_TARGET_PROPERTIES(GameOfLyfe PROPERTIES OUTPUT_NAME gol${BUILD_POSTFIX})
