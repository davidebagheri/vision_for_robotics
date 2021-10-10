cmake_minimum_required(VERSION 3.20.1)

project(excercise_1)

find_package(OpenCV REQUIRED)

include_directories(
	include
	${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
	src/utils.cpp
)

add_executable( show_cube show_cube.cpp)
target_link_libraries(show_cube ${OpenCV_LIBS} ${PROJECT_NAME})
