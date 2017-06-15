# robot_hw_sim_latency
Custom gazebo_ros_control simulation plugin that can add latency to communication between ros_control and gazebo

# Description

## gazebo_ros_control

Modified [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_ros_control) 
In order to allow modifying the time and period in a latency plugin, changed the readSim(ros::Time& time, ros::Duration& period) function to use references.

## robot_hw_sim_latency

Modification of [default_robot_hw_sim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp) plugin to allow loading custom plugins that can add latency between ros_control and gazebo.

## latency_plugin_simple_queue

Simple example latency plugin using queues
