cmake_minimum_required(VERSION 2.8.3)
project(urdf_std_format_converter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        cmake_modules
        urdf
        roscpp)

find_package(Eigen3 REQUIRED)

find_package(TinyXML REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urdf_std_format_converter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(converter
        src/converter.cpp)

target_link_libraries(converter
        ${catkin_LIBRARIES}
        ${TinyXML_LIBRARIES}
        ${EIGEN3_LIBRARIES})

