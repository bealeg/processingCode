cmake_minimum_required(VERSION 3.0.2)
project(hd_map_stf)

find_package(autoware_build_flags REQUIRED)

set(catkin_deps
    roscpp
    roslib
    tf2
    tf2_ros
    roslint
)
find_package(catkin REQUIRED ${catkin_deps})

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS ${catkin_deps}
)

###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hd_map_stf src/hd_map_stf.cpp)
target_link_libraries(hd_map_stf
    ${catkin_LIBRARIES}
    libGeographic.so
)

#############
## Install ##
#############

install(
  TARGETS hd_map_stf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

