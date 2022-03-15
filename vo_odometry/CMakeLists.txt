project(vo_odometry)

add_library(${PROJECT_NAME} SHARED
    src/vo_system.cpp
    src/tracker.cpp
    src/initializer.cpp
)
