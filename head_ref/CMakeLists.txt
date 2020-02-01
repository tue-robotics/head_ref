cmake_minimum_required(VERSION 2.8.3)
project(head_ref)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  head_ref_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  urdf
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS actionlib geometry_msgs sensor_msgs std_msgs visualization_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE HEADER_FILES include/*.h)

# Head ref lib
add_library(head_reference              src/HeadReference.cpp
                                        ${HEADERS})
target_link_libraries(head_reference    ${catkin_LIBRARIES})
add_dependencies(head_reference         ${catkin_EXPORTED_TARGETS})

# binary
add_executable(head_ref                 src/head_ref.cpp)
target_link_libraries(head_ref          head_reference)
add_dependencies(head_ref               ${catkin_EXPORTED_TARGETS})

# TESTS
add_executable(test_client              test/client.cpp)
target_link_libraries(test_client       head_reference)
add_dependencies(head_ref               ${catkin_EXPORTED_TARGETS})

# CLIENTS
add_executable(look_around_client           clients/look_around_client.cpp)
target_link_libraries(look_around_client    head_reference)

add_executable(cmd_vel_client               clients/cmd_vel_client.cpp)
target_link_libraries(cmd_vel_client        head_reference)
