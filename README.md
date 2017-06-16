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

1. To configure gazebo_ros_control to load robot_hw_sim_latency plugin instead of the default, modify your robot's urdf:

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


# Detailed working mechanism of the system

![Working mechanism of the system](https://github.com/szgezu/robot_hw_sim_latency/blob/master/system.png "Working mechanism of the system")

The working mechanism of the system is the following:
1. The gazebo_ros_control update function fires
2. it calls the readSim function; the call is executed in the RobotHWSimLatency plugin which implements the readSim function
3. the states are read from the gazebo internals
4. the delayStates function is called in the Simple queue latency plugin that saves the state messages in a buffer
5. the previously stored and now delayed states are returned from the Simple queue latency plugin to the RobotHWSimLatency plugin
6. readSim writes the joint_states to the JointStateInterface of the Harware Resource Interface Layer
7. gazebo_ros_control calls the update function of the controler_manager
8. the joint_trajectory_controller in the controller manager executes the calculation of the PID-controllers
9. the joint_trajectory_controller writes the calculated velocity commands to the VelocityInterface of the Hardware Resource Interface Layer
10. the gazebo_ros_control calls the writeSim function which is implemented in the RobotHWSimLatency plugin
11. the writeSim function reads the joint commands from the VelocityInterface of the Hardware Resource Interface Layer
12. the writeSim function calls the delayCommands function of the Simple queue latency plugin
13. the previously stored and now delayed commands are returned from the Simple queue latency plugin to the RobotHWSimLatency plugin
14. the writeSim function writes the joint commands to the gazebo internals
15. Gazebo calculate the internal states of the simulation loop
16. a new simulation loop is started by calling the update function of the gazebo_ros_control plugin


# Expected output

![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory2.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory3.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory4.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory5.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory6.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory7.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory8.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory9.png "Same trajectories with various latency settings")



