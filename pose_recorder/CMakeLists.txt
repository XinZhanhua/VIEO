cmake_minimum_required(VERSION 2.8.3)
project(pose_recorder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_recorder src/pose_recorder.cpp)
target_link_libraries(pose_recorder ${catkin_LIBRARIES})

install(TARGETS pose_recorder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
