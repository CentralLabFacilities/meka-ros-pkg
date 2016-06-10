cmake_minimum_required(VERSION 2.8.3)
project(hri_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grasping_msgs
  moveit_msgs
  trajectory_msgs
)

add_message_files(
  FILES
  HriTask.msg
  HriTaskResult.msg
  HriTaskPhase.msg
  HriEvent.msg
  HriInteraction.msg
)

#add_service_files(
#  FILES
#
#)

add_action_files(FILES Hri.action)

generate_messages(   
  DEPENDENCIES  
  geometry_msgs
  grasping_msgs
  moveit_msgs
  trajectory_msgs
)  

catkin_package(
  CATKIN_DEPENDS message_runtime )
