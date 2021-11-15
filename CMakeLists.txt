cmake_minimum_required(VERSION 3.0.2)
project(cam_radar_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_filters
)

find_package(PCL 1.2 REQUIRED)

SET(OpenCV_DIR /home/dd/repos/opencv/installation/OpenCV-master/lib/cmake/opencv4)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cam_radar_calibration
 CATKIN_DEPENDS cv_bridge eigen_conversions geometry_msgs image_transport pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs message_filters
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node src/radar_camera_reprojection.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)