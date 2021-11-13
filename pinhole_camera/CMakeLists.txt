cmake_minimum_required(VERSION 3.20.1)

project(pinhole_camera)

add_library(${PROJECT_NAME} SHARED
	src/camera_operations.cpp
)