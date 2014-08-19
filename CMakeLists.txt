cmake_minimum_required(VERSION 2.8.3)
project(head_ref)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  message_generation
)

# Generate actions in the 'action' folder
add_action_files(
   FILES
   HeadReference.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs std_msgs 
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
add_dependencies(head_reference         ${PROJECT_NAME}_gencpp)

# binary
add_executable(head_ref                 src/head_ref.cpp)
target_link_libraries(head_ref          head_reference)
add_dependencies(head_ref               ${PROJECT_NAME}_gencpp)

# TESTS
add_executable(test_client              test/client.cpp)
target_link_libraries(test_client       head_reference)
add_dependencies(head_ref               ${PROJECT_NAME}_gencpp)
