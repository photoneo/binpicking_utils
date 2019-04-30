cmake_minimum_required(VERSION 3.12)
project(pho_centroid)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.7.2 EXACT PATHS "${ThirdPartyDIR}/pcl-1.8.0/share/pcl-1.8")

set(OpenCV_DIR "${ThirdPartyDIR}/opencv-3.1.0-without_cuda/share/OpenCV")
FIND_PACKAGE(OpenCV REQUIRED)

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(pho_centroid main.cpp)

target_link_libraries(
        ${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

