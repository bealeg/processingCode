cmake_minimum_required(VERSION 2.8.3)
project(lidar_euclidean_track)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  autoware_msgs
  jsk_recognition_msgs
  std_msgs
  tf
  )


set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  autoware_msgs
  jsk_recognition_msgs
  std_msgs
  tf
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

#Euclidean Track
add_executable(lidar_euclidean_track
  nodes/lidar_euclidean_track/lidar_euclidean_track.cpp
  )
target_link_libraries(lidar_euclidean_track
  ${catkin_LIBRARIES}
  )
add_dependencies(lidar_euclidean_track
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS
		lidar_euclidean_track
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
