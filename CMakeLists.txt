cmake_minimum_required(VERSION 2.8.3)
project(vicon_depthcam_calib)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_executable(pointcloud_to_global src/pointcloud_to_global.cpp)
target_link_libraries(pointcloud_to_global ${catkin_LIBRARIES})

add_executable(vicon_checker_calib src/vicon_checker_calib.cpp)
target_link_libraries(vicon_checker_calib ${catkin_LIBRARIES})

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})


