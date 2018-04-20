cmake_minimum_required(VERSION 2.8.3)
project(localization_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    sensor_msgs
    message_generation
    cv_bridge
    tf
)

find_package(OpenCV REQUIRED core highgui videoio)
find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

get_cmake_property(_variableNames CACHE_VARIABLES)
foreach (_variableName ${_variableNames})
    if(_variableName MATCHES "^[bB][oO][oO][sS][tT]")
        message("unsetting ${_variableName}")
        unset(${_variableName} CACHE)
    endif()
endforeach()
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    if(_variableName MATCHES "^[bB][oO][oO][sS][tT]")
        message("unsetting ${_variableName}")
       unset(${_variableName})
   endif()
endforeach()

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(PhoLocalization REQUIRED CONFIG PATHS "$ENV{PHO_LOCALIZATION_PATH}")

set(PHO_ALPHAROOT $ENV{HOME})
set(ThirdPartyDIR "${PHO_ALPHAROOT}/3rdParty_REPO_gcc${CMAKE_CXX_COMPILER_VERSION}")
set(BOOST_INCLUDEDIR "${ThirdPartyDIR}/boost_1_61_0/include" CACHE PATH "x")
set(BOOST_LIBRARYDIR "${ThirdPartyDIR}/boost_1_61_0/lib" CACHE PATH "x")
set(BOOST_DLL_DIR "${BOOST_INCLUDEDIR}/../lib")
set(PCL_DIR "${ThirdPartyDIR}/pcl-1.8.0/share/pcl-1.8")
set(Boost_USE_STATIC_LIBS ON)


find_package(Boost 1.61.0 EXACT REQUIRED COMPONENTS
        system
        filesystem
        thread
        date_time
        iostreams
        serialization
        chrono
        atomic
        regex
        log
        python
)


add_service_files(FILES get_position.srv)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs    
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PHOLOCALIZATION_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(localization_interface src/localization_interface.cpp)
add_dependencies(localization_interface localization_interface_generate_messages_cpp)
string(REGEX REPLACE "/usr/lib/x86_64-linux-gnu/libboost"
        "${BOOST_LIBRARYDIR}/libboost" catkin_LIBRARIES
        "${catkin_LIBRARIES}")

string(REGEX REPLACE "${BOOST_LIBRARYDIR}/libboost_signals" "/usr/lib/x86_64-linux-gnu/libboost_signals"
        catkin_LIBRARIES "${catkin_LIBRARIES}")

target_link_libraries(localization_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PHOLOCALIZATION_LIBRARIES} ${OpenCV_LIBRARIES})
