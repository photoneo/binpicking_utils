cmake_minimum_required(VERSION 2.8.3)
project(bin_pose_emulator)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    bin_pose_msgs
    tf
   sensor_msgs
   pho_localization_msgs
   actionlib
   actionlib_msgs
   sensor_msgs

        )

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

add_executable(
  ${PROJECT_NAME}
        src/BinPoseEmulator.cpp
        src/BinPoseEmulatorNode.cpp
        src/ActionServerInterface.cpp
        src/pose_generator/PoseGeneratorBase.cpp
        src/pose_generator/PoseGeneratorFromCube.cpp
        src/pose_generator/PoseGeneratorFromCubeRandom.cpp
        src/pose_generator/PoseGeneratorFromPointCloud.cpp
        src/pose_generator/PoseGeneratorFromPointCloudRandom.cpp
        src/pose_generator/SinglePoseGenerator.cpp
)

  
add_dependencies(bin_pose_emulator ${catkin_EXPORTED_TARGETS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
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

