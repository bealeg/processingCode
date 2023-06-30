cmake_minimum_required(VERSION 3.0.2)
project(tf_pointcloud)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  tf2_py
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_pointcloud
   CATKIN_DEPENDS geometry_msgs rospy sensor_msgs tf2_py tf2_ros tf2_sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosbag_work_cpp src/rosbag_work_cpp.cpp)
add_executable(rosbag_work_cpp_simple src/rosbag_work_cpp_simple.cpp)
target_link_libraries(rosbag_work_cpp
    ${catkin_LIBRARIES}
)
target_link_libraries(rosbag_work_cpp_simple
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS rosbag_work_cpp rosbag_work_cpp_simple
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/tf_pointcloud.py
  scripts/rosbag_work.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
