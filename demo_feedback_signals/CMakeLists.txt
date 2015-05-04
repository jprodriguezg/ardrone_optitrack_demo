cmake_minimum_required(VERSION 2.8.3)
project(drone_feedback_signals)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sound_play
  message_generation
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   demo_sounds.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(leds_feedback_signals_node src/leds_feedback_signals.cpp)
add_dependencies(leds_feedback_signals_node drone_control_msgs_generate_messages_cpp)
target_link_libraries(leds_feedback_signals_node ${catkin_LIBRARIES})


add_executable(demo_sounds_server src/demo_sounds_server.cpp)
add_dependencies(demo_sounds_server drone_control_msgs_generate_messages_cpp)
target_link_libraries(demo_sounds_server ${catkin_LIBRARIES})

#############
## Install ##
#############



#############
## Testing ##
#############


