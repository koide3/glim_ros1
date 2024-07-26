cmake_minimum_required(VERSION 3.0.2)
project(glim_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport tf2_ros roslib rosbag sensor_msgs)

find_package(glim REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED core)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)

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

if(BUILD_WITH_VIEWER)
  add_definitions(-DBUILD_WITH_VIEWER)
endif()

if(BUILD_WITH_CUDA)
  add_definitions(-DBUILD_GTSAM_POINTS_GPU)
endif()

add_library(glim_ros
  src/glim_ros/glim_ros.cpp
)
target_include_directories(glim_ros PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(glim_ros
  spdlog::spdlog
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(rviz_viewer SHARED
  src/glim_ros/rviz_viewer.cpp
)
target_link_libraries(rviz_viewer
  glim_ros
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


add_executable(offline_viewer
  src/offline_viewer.cpp
)
target_link_libraries(offline_viewer
  interactive_viewer
  glim_ros
)

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS glim_ros rviz_viewer glim_rosnode glim_rosbag offline_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
