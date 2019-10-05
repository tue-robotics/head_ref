cmake_minimum_required(VERSION 2.8.3)
project(head_ref_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  std_msgs
  message_generation
)

# Generate actions in the 'action' folder
add_action_files(
   FILES
   HeadReference.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime std_msgs
)
