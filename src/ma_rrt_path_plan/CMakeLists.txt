cmake_minimum_required(VERSION 2.8.3)
project(ma_rrt_path_plan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  message_generation
)

## Declare ROS messages
add_message_files(
  FILES
    Waypoint.msg
    WaypointsArray.msg
    Map.msg
)

## Generate added messages and services with any dependencies
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Install launch files and waypoint definitions
install(DIRECTORY
  launch
  waypoints
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install Python nodes and scripts
install(PROGRAMS
  src/cone_map_adapter.py
  src/main.py
  src/MaRRTPathPlanNode.py
  src/inc/ma_rrt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
