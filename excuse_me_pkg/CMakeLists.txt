cmake_minimum_required(VERSION 3.0.2)
project(excuse_me_pkg)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  autoware_msgs
  pcl_ros
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
    autoware_msgs
    tf
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dumpTextSample src/dumpTextSample.cpp)
target_link_libraries(dumpTextSample
    ${catkin_LIBRARIES}
    libGeographic.so
)
# target_link_libraries(excues_me_process
#  ${catkin_LIBRARIES}
#)
# add_dependencies(excuse_me_process
#   ${catkin_EXPORTED_TARGETS}
# )

install(
  TARGETS dumpTextSample# excuse_me_process
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(
  DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE
)
