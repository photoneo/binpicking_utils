cmake_minimum_required(VERSION 2.8.3)
project(bin_pose_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  moveit_msgs
  geometry_msgs
  message_generation
  genmsg
)

add_service_files(
  FILES
  bin_pose.srv
  joint_limits.srv
  envelope_visualize.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs moveit_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs moveit_msgs genmsg
)
