cmake_minimum_required(VERSION 3.20.1)
project(vision_for_robotics)

find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
    utils/include
    excercise_1/include
    PnP/include
    simple_keypoint_tracker/include
)

# Utils 
add_subdirectory(utils)

# Excercise 1 - Camera operations
add_subdirectory(excercise_1)

add_executable(show_cube excercise_1/show_cube.cpp)
target_link_libraries(show_cube ${OpenCV_LIBS} excercise_1 utils)

add_executable(undistort_image excercise_1/undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS} excercise_1 utils)

# Excercise 2 - PnP
add_subdirectory(PnP)

add_executable(estimate_pose PnP/estimate_pose.cpp)
target_link_libraries(estimate_pose ${OpenCV_LIBS} PnP utils excercise_1)

# Excercise 3 - Simple Keypoint Tracker
add_subdirectory(simple_keypoint_tracker)

add_executable(track_video simple_keypoint_tracker/track_video.cpp)
target_link_libraries(track_video ${OpenCV_LIBS} utils simple_keypoint_tracker)




