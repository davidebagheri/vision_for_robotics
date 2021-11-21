project(eight_point_algorithm)

add_library(${PROJECT_NAME} SHARED
    src/linear_triangulator.cpp
    src/eight_point_algorithm.cpp
)

