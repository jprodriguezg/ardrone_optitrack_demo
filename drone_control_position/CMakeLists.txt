cmake_minimum_required(VERSION 2.8.3)
project(drone_control_position)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  roscpp
  ardrone_autonomy
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(	CATKIN_DEPENDS
		roscpp 
		std_msgs
		geometry_msgs
		ardrone_autonomy
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(control_drone_position_node src/control_node.cpp)
add_dependencies(control_drone_position_node drone_control_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(control_drone_position_node ${catkin_LIBRARIES})

