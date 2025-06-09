cmake_minimum_required(VERSION 3.0.2)
project(mpc_control)

## Ensure standard headers are available to IPOPT
add_definitions(-DHAVE_STDDEF_H)

## Compile as C++11
add_compile_options(-std=c++11)

## Find Catkin and components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  ma_rrt_path_plan
  message_runtime
)

## Find Eigen
find_package(Eigen3 REQUIRED)

## Locate IPOPT library
find_library(IPOPT_LIBRARIES
  NAMES ipopt
  HINTS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
)
if (NOT IPOPT_LIBRARIES)
  message(FATAL_ERROR "libipopt.so not found. Install coinor-libipopt-dev.")
endif()

## Locate CppAD library (it may be named cppad_lib on some installs)
find_library(CPPAD_LIBRARIES
  NAMES cppad libcppad cppad_lib
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
)
if (NOT CPPAD_LIBRARIES)
  message(FATAL_ERROR "libcppad not found. Ensure CppAD is built and installed (sudo make install).")
endif()

## Declare Catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    ackermann_msgs
    ma_rrt_path_plan
    message_runtime
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/coin     # IPOPT headers
  /usr/local/include    # CppAD headers
)

## MPC solver library
add_library(mpc_solver
  src/MPC.cpp
)
target_link_libraries(mpc_solver
  ${IPOPT_LIBRARIES}
  ${CPPAD_LIBRARIES}
)

## MPC node executable
add_executable(mpc_node
  src/mpc_node.cpp
)
target_link_libraries(mpc_node
  mpc_solver
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS mpc_node mpc_solver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
