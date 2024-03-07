cmake_minimum_required(VERSION 3.16.3)

project(pinhole_camera)

add_library(${PROJECT_NAME} SHARED
	src/camera_operations.cpp
)