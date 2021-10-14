cmake_minimum_required(VERSION 3.20.1)
project(vision_for_robotics)

find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
    utils/include
    excercise_1/include
)

# Utils 
add_subdirectory(utils)

# Excercise 1 - Camera operations
add_subdirectory(excercise_1)

add_executable(show_cube excercise_1/show_cube.cpp)
target_link_libraries(show_cube ${OpenCV_LIBS} excercise_1 utils)

add_executable(undistort_image excercise_1/undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS} excercise_1 utils)


