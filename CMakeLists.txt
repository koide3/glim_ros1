cmake_minimum_required(VERSION 3.0.2)
project(glim_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport tf2_ros roslib rosbag sensor_msgs nmea_msgs)

find_package(glim REQUIRED)
find_package(glim_ext QUIET)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES glim_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

if(glim_ext_FOUND)
  add_definitions(-DBUILD_WITH_GLIM_EXT)
endif()

if(BUILD_WITH_VIEWER)
  add_definitions(-DBUILD_WITH_VIEWER)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${glim_ext_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(glim_ros
  src/glim_ros/glim_ros.cpp
  src/glim_ros/rviz_viewer.cpp
)
target_link_libraries(glim_ros
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  ${glim_ext_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(glim_rosnode
  src/glim_rosnode.cpp
)
target_link_libraries(glim_rosnode
  glim_ros
)

add_executable(glim_rosbag
  src/glim_rosbag.cpp
)
target_link_libraries(glim_rosbag
  glim_ros
)

if(BUILD_WITH_VIEWER)
  add_executable(offline_viewer
    src/offline_viewer.cpp
  )
  target_link_libraries(offline_viewer
    ${glim_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endif()