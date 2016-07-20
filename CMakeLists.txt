cmake_minimum_required(VERSION 2.8.3)
project(vicon_depthcam_scan)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  minkindr
  minkindr_conversions
 image_transport
)

find_package(Eigen3 REQUIRED)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES vicon_depthcam_scan
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp sensor_msgs image_transport
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(pointcloud_to_global src/pointcloud_to_global.cpp)
target_link_libraries(pointcloud_to_global ${catkin_LIBRARIES})

add_executable(ircamera_tf src/ircamera_tf.cpp)
target_link_libraries(ircamera_tf ${catkin_LIBRARIES})

add_executable(pointcloud_preprocess src/pointcloud_preprocess.cpp)
target_link_libraries(pointcloud_preprocess ${catkin_LIBRARIES})

add_executable(test_extrinsic_calibration src/test_extrinsic_calibration.cpp)
target_link_libraries(test_extrinsic_calibration ${catkin_LIBRARIES})

add_executable(transform_calc src/transform_calc.cpp)
target_link_libraries(transform_calc ${catkin_LIBRARIES})

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})
