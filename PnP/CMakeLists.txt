project(PnP)
cmake_minimum_required(VERSION 3.16.3)

add_library(${PROJECT_NAME} SHARED
    src/PnP.cpp
)