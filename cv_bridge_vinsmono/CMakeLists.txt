cmake_minimum_required(VERSION 3.0.2)
project(cv_bridge_vinsmono)

find_package(catkin REQUIRED COMPONENTS rosconsole sensor_msgs)

if(NOT ANDROID)
  find_package(PythonLibs)

  if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    # Debian Buster
    find_package(Boost REQUIRED python37)
  else()
    # Ubuntu Focal
    find_package(Boost REQUIRED python)
  endif()
else()
find_package(Boost REQUIRED)
endif()

set(OpenCV_DIR /home/siae302/ku/opencv3.4.1/opencv-3.4.1/build)
set(_opencv_version 3)
find_package(OpenCV 3.4.1 EXACT)

if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 3, trying OpenCV 3 again, see cv_brige_vinsmono")
  set(_opencv_version 3)
endif()

find_package(OpenCV ${_opencv_version} REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosconsole sensor_msgs
  DEPENDS OpenCV
  CFG_EXTRAS cv_bridge-extras.cmake
)

catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

if(NOT ANDROID)
add_subdirectory(python)
endif()
add_subdirectory(src)
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
