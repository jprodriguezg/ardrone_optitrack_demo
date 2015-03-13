cmake_minimum_required(VERSION 2.8.3)
project(drone_following_leader)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(following_leader_node src/following_leader.cpp)
add_executable(leader_selector_node src/leader_selector.cpp)
add_executable(leader_desired_position_node src/leader_position.cpp)

add_dependencies(following_leader_node drone_control_msgs_generate_messages_cpp)
add_dependencies(leader_selector_node drone_control_msgs_generate_messages_cpp)
add_dependencies(leader_desired_position_node drone_control_msgs_generate_messages_cpp)

target_link_libraries(following_leader_node ${catkin_LIBRARIES})
target_link_libraries(leader_selector_node ${catkin_LIBRARIES})
target_link_libraries(leader_desired_position_node ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############


