# robot_hw_sim_latency
Custom gazebo_ros_control simulation plugin that can add latency to communication between ros_control and gazebo

# Description


## robot_hw_sim_latency

Modification of the [default_robot_hw_sim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp) plugin to allow loading custom plugins that can add latency between ros_control and gazebo.

## latency_plugin_simple_queue

Simple example latency plugin using queues


# Installation

I assume a catkin workspace is already set up, and gazebo_ros_pkgs is [installed](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros). 

1. Copy the two folders(robot_hw_sim_latency, latency_plugin_simple_queue) to your workspace
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

3. To configure the length of queue in latency_plugin_simple_queue, set the `/latency_plugin_simple_queue/queue_length` parameter on the Parameter Server.

  Example using roslaunch:

 ```xml
   <rosparam param="/latency_plugin_simple_queue/queue_length">10</rosparam>
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

# How to cite?
```xml
@article{gazebopluginconf,
author = {G. Szabo and S. Racz and J. Peto and R. R. Aschoff},
title = {On The Effects of The Variations In Network Characteristics In Cyber Physical Systems},
journal = {In Proc., 31st European Simulation and Modelling Conference},
year = {2017},
month = {Oct.}
location = {Lisbon, Portugal}
}
```

# Expected output

![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory2.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory3.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory4.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory5.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory6.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory7.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory8.png "Same trajectories with various latency settings")
![Same trajectories with various latency settings](https://github.com/szgezu/robot_hw_sim_latency/blob/master/trajectory9.png "Same trajectories with various latency settings")

# Digital Twin mode
## Web demo
### Installation
1. Download and extract `_digital_twin_demo.tar.gz` from Digital twin demo in [Files | https://forge.ericsson.net/file/showfiles.php?group_id=576]
2. In the folder run `catkin_make`, then `catkin_make install`
3. If failed install dependencies
4. Then `source install/setup.bash` in every opened terminal you intend to use

## Running
1. In a terminal run `printf 'movej(~[3.14,-1.13,1.51,3.77,-1.51,0.0])\nend' | nc <ip.of.robot> 30002 &>/dev/null`
2. Run `export SUDO_PASSWORD=<sudo password>`
3. Run `cd /the/folder/you/extracted/to>`
4. Run `source install/setup.bash`
5. Run `roslaunch digital_twin_demo setup.launch robot_ip:=<ip.of.robot>`
6. Open `http://localhost:8000/demo.html` in Chrome or Chromium


### Running demo with ursim
1. Run `roslaunch osrf_gear figment_environment.launch competition:=finals/final01.yaml` in a terminal
2. Run `gz world -m 4` to advance simulation by 4 ticks to work around some race conditions
3. Run `rosrun ur5_vel_start run_in_network_namespace_with_latency.py "sudo -u user /home/user/ursim-3.5.3.10825/start-ursim.sh"=, if you want to set latency add =--delay <number>` to set it
4. If you don't want to create a separate network namespace ou can run `start-ursim.sh` but then change IP in next step to 127.0.0.1
5. Run `roslaunch ur5_vel_start setup.launch robot_ip:=10.0.1.2 sim:=false ns:=realur/ moveit:=false`
6. Start simulation using gazebo gui
7. Run `rosrun figment_ariac scheduler_plan`

## To turn on/off QoC control of figment controller
1. Open 'arm_actions.py' file in figment_ariac/gprt folder
2. Uncomment/comment lines 'set_latency(20 if value=='Low' else 1)' in functions 'set_trajectory_precision' and `set_trajectory_precision_value` and change device in line `change_latency_cmd~='sudo -S tc qdisc change dev enp5s0 root netem  delay {delay}ms'</code> in =init()`
3. Run `catkin_make install`
4. Changing of latency will only work if a `sudo tc qdisc change dev enp5s0 root netem  delay {delay}ms` command was issued before running figment_ariac

