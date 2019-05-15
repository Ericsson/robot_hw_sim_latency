# robot_hw_sim_latency
Custom gazebo_ros_control simulation plugin that can add latency to communication between ros_control and gazebo

Modification of the [default_robot_hw_sim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp) plugin to allow loading custom plugins that can add latency between ros_control and gazebo.

# Digital Twin mode

## Installation
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

## Running demo with ursim
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

## Notes on the working mechanism
### Running a robot in a separate ROS namespace

In order to make it possible to control the 2 robot arms separately and without errors that are caused by creating a second robot arm with the same joint names (the startup process complains if there are multiple joints with the same name) I had to create a method that changes the names of joints and other ROS parameters.

We made a modification to my roslaunch script that handles starting the whole UR system so I can pass a parameter to it (ns:=) that makes it load all configuration in a ROS namespace:

`roslaunch ur5_vel_start setup.launch robot_ip:=10.0.1.2 sim:=false ns:=realur/ moveit:=false`

Also, because the joint names are loaded from a yaml file we needed to a way to dynamically modify a yaml configuration, roslaunch provides such a method ([http://wiki.ros.org/roslaunch/XML#substitution_args]), but it only works with yaml configuration embedded in roslaunch files. I modified roslaunch to use this substitution method with yaml configuration loaded from files:

```xml
ur5_controllers.yaml:
ur5_velocity_controller:
   type: velocity_controllers/~JointGroupVelocityController
   joints:
     - $(arg ns)shoulder_pan_joint
     - $(arg ns)shoulder_lift_joint
     - $(arg ns)elbow_joint
     - $(arg ns)wrist_1_joint
     - $(arg ns)wrist_2_joint
     - $(arg ns)wrist_3_joint
```

We also created a pull request to the roslaunch ~GitHub repository with this change, and it was accepted: [https://github.com/ros/ros_comm/pull/1354]

This way we can control the simulated and real UR arms independently.

![Real UR5 rqt graph](https://github.com/Ericsson/robot_hw_sim_latency/blob/Digital-Twin----Kinetic/realur.png "Real UR5 rqt graph")

ROS Param configurations
```xml
/realur/controller_joint_names
/realur/force_torque_sensor_controller/publish_rate
/realur/force_torque_sensor_controller/type
/realur/hardware_control_loop/loop_hz
/realur/hardware_interface/joints
/realur/joint_state_controller/publish_rate
/realur/joint_state_controller/type
/realur/robot_description
/realur/ur5_velocity_controller/joints
/realur/ur5_velocity_controller/type
/realur/ur5_velocity_controller_moveit/action_monitor_rate
/realur/ur5_velocity_controller_moveit/constraints/goal_time
/realur/ur5_velocity_controller_moveit/constraints/realur/elbow_joint/goal
/realur/ur5_velocity_controller_moveit/constraints/realur/elbow_joint/trajectory
/realur/ur5_velocity_controller_moveit/constraints/realur/shoulder_lift_joint/goal
/realur/ur5_velocity_controller_moveit/constraints/realur/shoulder_lift_joint/trajectory
/realur/ur5_velocity_controller_moveit/constraints/realur/shoulder_pan_joint/goal
/realur/ur5_velocity_controller_moveit/constraints/realur/shoulder_pan_joint/trajectory
/realur/ur5_velocity_controller_moveit/constraints/realur/wrist_1_joint/goal
/realur/ur5_velocity_controller_moveit/constraints/realur/wrist_1_joint/trajectory
/realur/ur5_velocity_controller_moveit/constraints/realur/wrist_2_joint/goal
/realur/ur5_velocity_controller_moveit/constraints/realur/wrist_2_joint/trajectory
/realur/ur5_velocity_controller_moveit/constraints/realur/wrist_3_joint/goal
/realur/ur5_velocity_controller_moveit/constraints/realur/wrist_3_joint/trajectory
/realur/ur5_velocity_controller_moveit/constraints/stopped_velocity_tolerance
/realur/ur5_velocity_controller_moveit/gains/realur/elbow_joint/d
/realur/ur5_velocity_controller_moveit/gains/realur/elbow_joint/i
/realur/ur5_velocity_controller_moveit/gains/realur/elbow_joint/i_clamp
/realur/ur5_velocity_controller_moveit/gains/realur/elbow_joint/p
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_lift_joint/d
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_lift_joint/i
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_lift_joint/i_clamp
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_lift_joint/p
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_pan_joint/d
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_pan_joint/i
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_pan_joint/i_clamp
/realur/ur5_velocity_controller_moveit/gains/realur/shoulder_pan_joint/p
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_1_joint/d
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_1_joint/i
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_1_joint/i_clamp
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_1_joint/p
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_2_joint/d
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_2_joint/i
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_2_joint/i_clamp
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_2_joint/p
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_3_joint/d
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_3_joint/i
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_3_joint/i_clamp
/realur/ur5_velocity_controller_moveit/gains/realur/wrist_3_joint/p
/realur/ur5_velocity_controller_moveit/joints
/realur/ur5_velocity_controller_moveit/state_publish_rate
/realur/ur5_velocity_controller_moveit/stop_trajectory_duration
/realur/ur5_velocity_controller_moveit/type
/realur/ur_hardware_interface/max_payload
/realur/ur_hardware_interface/max_vel_change
/realur/ur_hardware_interface/max_velocity
/realur/ur_hardware_interface/min_payload
/realur/ur_hardware_interface/prefix
/realur/ur_hardware_interface/robot_ip_address
/realur/ur_hardware_interface/use_ros_control
```

## latency_plugin_velocity_control inner workings

To control the UR5 robot using the latency plugin infrastructure we did the following:
1. In the initPlugin function, I create a ros node that can publish and subscribe to ros topics.
2. I subscribe to a topic through which I will receive joint states from the real UR arm, and register its callback function
3. And I register a topic to which i will send velocity comands.

In the delayStates function I do nothing, just pass through the unmodified simulation joint state.
In the callback function that handles the joint state messages I save the joint positions and velocities (joint states), and through a feed forward PID controller I compute a velocity command that I publish, this way I send a control message every time I receive a state message from the real robot.
The controller's input is the position and velocity command that the delayCommand function received.

The delayCommand function controls the simulated robot also using a feed forward PID controller the inputs being the joint states.
Because the delayCommand function is called much more frequently than the callbach I implemented simple interpolation between position joint states.
I do not directly use the position joint state, but I add the velocity joint state to it every time the delayCommand function runs, but the callback not, therefore every subsequent delayCommand call the position increases by the velocity * dt even when it wouldn't change because the callback was not called, this avoids abrupt changes in the position command sent by the delayCommands function.
Also this function also saves the current velocity and position commands that the callback uses.

# Expected output
[Demonstration video](https://youtu.be/CdFQXK04Z_I)
![System overview](https://github.com/Ericsson/robot_hw_sim_latency/blob/Digital-Twin----Kinetic/DTsystem.png "System overiview")

# How to cite?
```xml
@article{dtpluginconf,
author = {G. Szabo and S. Racz and N. Reider and J. Peto},
title = {Digital Twin: Network Provisioning of Mission Critical Communication in Cyber Physical Production Systems},
journal = {In Proc., The IEEE International Conference on Industry 4.0, Artificial Intelligence, and Communications Technology},
year = {2019},
month = {Jul.}
}
```


