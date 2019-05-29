cmake_minimum_required(VERSION 2.8.3)
project(latency_plugin_velocity_publisher)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_hw_sim_latency
  pluginlib
  control_toolbox
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED COMPONENTS thread)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_hw_sim_latency
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pluginlib
    robot_hw_sim_latency
  DEPENDS

  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    latency_plugin_velocity_publisher
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)



add_library(latency_plugin_velocity_publisher src/latency_plugin_velocity_publisher.cpp)
target_link_libraries(latency_plugin_velocity_publisher ${catkin_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME} latency_plugin_velocity_publisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES sim_latency_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
