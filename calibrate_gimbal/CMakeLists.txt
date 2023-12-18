cmake_minimum_required(VERSION 3.0.2)
project(calibrate_gimbal)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    camera_model
    cv_bridge_vinsmono
    apriltag_ros
    tf2
)

find_package(OpenCV 3 REQUIRED)



catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(calibrate_gimbal
    src/parameters.cpp
    src/calibrate_gimbal_node.cpp
    src/calibrate_gimbal.cpp
    )

target_link_libraries(calibrate_gimbal ${catkin_LIBRARIES} ${OpenCV_LIBS})
