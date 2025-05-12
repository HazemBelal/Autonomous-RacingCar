cmake_minimum_required(VERSION 2.8.3)
project(ma_rrt_path_plan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  message_generation
)

## Declare ROS messages, services and actions
add_message_files(
  FILES
  Waypoint.msg
  WaypointsArray.msg
  # (If you create a custom Map.msg, add it here as well)
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
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

install(DIRECTORY
    launch
    waypoints
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
    src/main.py
    src/MaRRTPathPlanNode.py
    src/inc/ma_rrt.py
    src/cone_map_adapter.py   # <-- Added adapter node here
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
