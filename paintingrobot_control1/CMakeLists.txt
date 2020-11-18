cmake_minimum_required(VERSION 2.8.3)
project(paintingrobot_control1)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  message_generation
)

find_package(yaml-cpp REQUIRED)
add_message_files(
  FILES
  catersian_vel.msg
  physical_para.msg
#   Message1.msg
#   Message2.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES painting_robot_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



