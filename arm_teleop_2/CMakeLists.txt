cmake_minimum_required(VERSION 3.0.2)
project(arm_teleop_2)

find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  moveit_msgs
  geometry_msgs
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_group_python_interface_tutorial
  CATKIN_DEPENDS moveit_commander moveit_msgs geometry_msgs rospy std_msgs
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)