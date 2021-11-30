cmake_minimum_required(VERSION 2.8.3)
project(visualize_subject_vehicle_velo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  )


set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  std_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

#
add_executable(visualize_subject_vehicle_velo
  src/visualizeSubjectVehicleVelo.cpp
  )
target_link_libraries(visualize_subject_vehicle_velo
  ${catkin_LIBRARIES}
  )
add_dependencies(visualize_subject_vehicle_velo
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS
		visualize_subject_vehicle_velo
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
