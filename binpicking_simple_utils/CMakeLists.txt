cmake_minimum_required(VERSION 2.8.3)
project(binpicking_simple_utils)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  bin_pose_msgs
  geometric_shapes
  joint_limits_interface
  tf
  tf_conversions
  pcl_conversions
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs moveit_msgs bin_pose_msgs geometric_shapes joint_limits_interface moveit_core moveit_ros_planning_interface tf
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(
  tool_pose_tf_broadcaster
  src/tool_pose_tf_broadcaster.cpp)

target_link_libraries(
  tool_pose_tf_broadcaster
  ${catkin_LIBRARIES})

add_executable(
  collision_object_publisher
  src/collision_object_publisher)

target_link_libraries(
  collision_object_publisher
  ${catkin_LIBRARIES}
  yaml-cpp)

add_executable(
  joint_limit_estimator
  src/joint_limit_estimator.cpp)

target_link_libraries(
  joint_limit_estimator
  ${catkin_LIBRARIES})

add_executable(
  working_envelope_visualizer
  src/working_envelope_visualizer.cpp)

target_link_libraries(
  working_envelope_visualizer
  ${catkin_LIBRARIES})

# binaries
install(TARGETS
  tool_pose_tf_broadcaster #collision_object_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# other files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
 





