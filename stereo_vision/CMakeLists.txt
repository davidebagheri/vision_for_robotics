project(stereo_vision)

set(CMAKE_CXX_FLAGS "-fopenmp")

add_library(${PROJECT_NAME} SHARED
    src/stereo_matcher.cpp
)