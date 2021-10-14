cmake_minimum_required(VERSION 3.20.1)

project(utils)

add_library(${PROJECT_NAME} SHARED
	src/conversion.cpp
)