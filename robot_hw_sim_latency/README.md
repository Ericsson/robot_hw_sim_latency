Modification of [default_robot_hw_sim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp) plugin to allow loading custom plugins that can add latency between ros_control and gazebo.

The default latency plugin (default_sim_latency_plugin) just copies the input to the output.
