cmake_minimum_required(VERSION 2.8.3)
project(binpicking_simple_utils)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  geometric_shapes
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs moveit_msgs geometric_shapes tf
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
 





