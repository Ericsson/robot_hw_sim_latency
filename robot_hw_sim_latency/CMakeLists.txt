cmake_minimum_required(VERSION 2.8.3)
project(robot_hw_sim_latency)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  gazebo_ros_control
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(GAZEBO REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

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
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
    gazebo_ros_control
  DEPENDS
    #GAZEBO  # Re-enable after issue resolved:
    # https://bitbucket.org/osrf/gazebo/issues/2198/gazebo7-linking-error-with-cmake-catkin
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    robot_hw_sim_latency
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)



add_library(robot_hw_sim_latency src/robot_hw_sim_latency.cpp src/default_sim_latency_plugin.cpp)
target_link_libraries(robot_hw_sim_latency ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME} robot_hw_sim_latency
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES sim_latency_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
