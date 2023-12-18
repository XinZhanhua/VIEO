cmake_minimum_required(VERSION 3.0.2)
project(gimbal_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial
  apriltag_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gimbal_control
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_tracking
    src/target_tracking.cpp
#    python/target_tracking.py
    )

target_link_libraries(target_tracking ${catkin_LIBRARIES})

catkin_package()
