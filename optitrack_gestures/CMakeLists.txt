cmake_minimum_required(VERSION 2.8.3)
project(optitrack_gestures)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sound_play
  ardrone_autonomy
  visual_object_detector
)


################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS ardrone_autonomy visual_object_detector

)

###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(takeoff_land_node src/takeoff-land.cpp)
add_executable(optitrack_gestures_demo src/demo.cpp)

add_dependencies(takeoff_land_node drone_control_msgs_generate_messages_cpp)
add_dependencies(optitrack_gestures_demo drone_control_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(takeoff_land_node ${catkin_LIBRARIES})
target_link_libraries(optitrack_gestures_demo ${catkin_LIBRARIES})


#############
## Install ##
#############



#############
## Testing ##
#############


