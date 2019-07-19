cmake_minimum_required(VERSION 2.8.3)
project(bin_pose_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  genmsg)

add_message_files(DIRECTORY msg
        FILES
        bin_pose_ms.msg
)

add_service_files(DIRECTORY srv
  FILES
  bin_pose.srv
  bin_pose_vector.srv
  change_data.srv)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)



catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs genmsg
)
