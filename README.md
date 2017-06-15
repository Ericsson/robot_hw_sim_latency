# robot_hw_sim_latency
Custom gazebo_ros_control simulation plugin that can add latency to communication between ros_control and gazebo

# Description

## gazebo_ros_control

Modified [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_ros_control) 
In order to allow modifying the time and period in a latency plugin, changed the readSim(ros::Time& time, ros::Duration& period) function to use references.

## robot_hw_sim_latency

Modification of the [default_robot_hw_sim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp) plugin to allow loading custom plugins that can add latency between ros_control and gazebo.

## latency_plugin_simple_queue

Simple example latency plugin using queues


# Installation

I assume a catkin workspace is already set up, and gazebo_ros_pkgs is [installed](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros). 

1. Copy the three folders to your workspace
2. Run `catkin_make`

# Usage

1. To configure gazebo_ros_control to load robot_hw_sim_latency plugin instead of the default modify your robot's urdf:

  Example: If using UR robots, change https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/urdf/common.gazebo.xacro

 ```xml
  <gazebo>
    <plugin name="ros_control" filename="libgazebo_ros_control.so">
      <!--robotNamespace>/</robotNamespace-->
      <robotSimType>robot_hw_sim_latency/RobotHWSimLatency</robotSimType>
      <!--robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType-->
    </plugin>
  </gazebo>
 ```
 
2. To configure robot_hw_sim_latency to load a custom latency plugin (like latency_plugin_simple_queue) instead of the default, set the `/robot_hw_sim_latency/latency_plugin` parameter on the Parameter Server.

  Example using roslaunch:

 ```xml
   <rosparam param="/robot_hw_sim_latency/latency_plugin">latency_plugin_simple_queue/SimpleQueueLatencyPlugin</rosparam>
 ```

