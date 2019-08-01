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
        ${catkin_INCLUDE_DIRS}
        include)

catkin_package(
    INCLUDE_DIRS include
#    LIBRARIES urdf_std_format_converter
#    CATKIN_DEPENDS other_catkin_pkg
#    DEPENDS system_lib
)

add_library(${PROJECT_NAME}
        src/urdf_std_format_converter.cpp)

add_executable(urdf_converter_node
        src/urdf_converter_node.cpp src/urdf_std_format_converter.cpp)

target_link_libraries(urdf_converter_node
        ${catkin_LIBRARIES}
        ${TinyXML_LIBRARIES}
        ${EIGEN3_LIBRARIES})

add_dependencies(
        urdf_converter_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})



