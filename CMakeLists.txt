cmake_minimum_required(VERSION 2.8.3)
project(ras_brain)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  #message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES  
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   geometry_msgs
# )
 


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ras_lab1_ds_calib
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(brain_node src/brain_node.cpp)
target_link_libraries(brain_node  ${catkin_LIBRARIES})

add_dependencies(brain_node  brain_msgs_generate_messages_cpp)

