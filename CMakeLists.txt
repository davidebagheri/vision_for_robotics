cmake_minimum_required(VERSION 3.16.3)
project(vision_for_robotics)

find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
    utils/include
    pinhole_camera/include
    PnP/include
    simple_keypoint_tracker/include
    stereo_vision/include
    eight_point_algorithm/include
    vo_odometry/include
)

# Utils 
add_subdirectory(utils)

# Excercise 1 - Camera operations
add_subdirectory(pinhole_camera)

add_executable(show_cube pinhole_camera/show_cube.cpp)
target_link_libraries(show_cube ${OpenCV_LIBS} pinhole_camera utils)

add_executable(undistort_image pinhole_camera/undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS} pinhole_camera utils)

# Excercise 2 - PnP
add_subdirectory(PnP)

add_executable(estimate_pose PnP/estimate_pose.cpp)
target_link_libraries(estimate_pose ${OpenCV_LIBS} PnP utils pinhole_camera)

# Excercise 3 - Simple Keypoint Tracker
add_subdirectory(simple_keypoint_tracker)

add_executable(track_video simple_keypoint_tracker/track_video.cpp)
target_link_libraries(track_video ${OpenCV_LIBS} utils simple_keypoint_tracker)

# Excercise 4 - Stereo Vision
add_subdirectory(stereo_vision)

add_executable(pointcloud_from_stero stereo_vision/pointcloud_from_stereo.cpp)
target_link_libraries(pointcloud_from_stero ${OpenCV_LIBS} utils stereo_vision)

# Excercise 5 - Eight Point Algorithm
add_subdirectory(eight_point_algorithm)

add_executable(test_eight_point_algo eight_point_algorithm/test_eight_point_algo.cpp)
target_link_libraries(test_eight_point_algo ${OpenCV_LIBS} eight_point_algorithm utils)

add_executable(run_eight_point_algo eight_point_algorithm/run_eight_point_algo.cpp)
target_link_libraries(run_eight_point_algo ${OpenCV_LIBS} eight_point_algorithm utils)

add_executable(triangulate_points eight_point_algorithm/triangulate_points.cpp)
target_link_libraries(triangulate_points ${OpenCV_LIBS} eight_point_algorithm utils)

# Final VO odometry
add_subdirectory(vo_odometry)

add_executable(main vo_odometry/src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS} vo_odometry utils)