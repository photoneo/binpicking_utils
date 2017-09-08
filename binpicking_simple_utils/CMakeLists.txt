cmake_minimum_required(VERSION 2.8.3)
project(binpicking_simple_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${catkin_INCLUDE_DIRS}  
)

add_executable(
  tool_pose_tf_broadcaster
  src/tool_pose_tf_broadcaster.cpp)

target_link_libraries(
  tool_pose_tf_broadcaster
  ${catkin_LIBRARIES})

# binaries
install(TARGETS
  tool_pose_tf_broadcaster
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# other files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
 





