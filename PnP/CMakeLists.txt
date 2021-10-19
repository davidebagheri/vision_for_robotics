project(PnP)
cmake_minimum_required(VERSION 3.20.1)

add_library(${PROJECT_NAME} SHARED
    src/PnP.cpp
)