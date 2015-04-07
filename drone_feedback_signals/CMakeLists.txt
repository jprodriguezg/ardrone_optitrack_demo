cmake_minimum_required(VERSION 2.8.3)
project(drone_feedback_signals)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sound_play
)


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
add_executable(feedback_signals_node src/feedback_signals.cpp)
add_dependencies(feedback_signals_node drone_control_msgs_generate_messages_cpp)
target_link_libraries(feedback_signals_node ${catkin_LIBRARIES})


#############
## Install ##
#############



#############
## Testing ##
#############


