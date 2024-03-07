cmake_minimum_required(VERSION 3.16.3)

project(utils)

add_library(${PROJECT_NAME} SHARED
	src/conversion.cpp
	src/visualizer.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})



