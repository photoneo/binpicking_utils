cmake_minimum_required(VERSION 2.8.3)
project(bin_pose_emulator)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    bin_pose_msgs
    tf)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/)

add_executable(
  ${PROJECT_NAME}
  src/bin_pose_emulator.cpp)
  
add_dependencies(bin_pose_emulator ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp)

# binaries
install(TARGETS
  bin_pose_emulator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# other files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

