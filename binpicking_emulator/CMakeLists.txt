cmake_minimum_required(VERSION 2.8.3)
project(binpicking_emulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    photoneo_msg
    bin_pose_msgs
    moveit_core
    moveit_ros_planning_interface
    tf)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp bin_pose_msgs photoneo_msg moveit_core)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/)

add_executable(
  binpicking_emulator
  src/binpicking_emulator.cpp)

add_dependencies(binpicking_emulator ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  binpicking_emulator
  ${catkin_LIBRARIES}
)

# binaries
install(TARGETS
  binpicking_emulator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# other files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
