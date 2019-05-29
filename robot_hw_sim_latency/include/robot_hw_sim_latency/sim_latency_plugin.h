#pragma once

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tuple>


namespace robot_hw_sim_latency
{
class SimLatencyPlugin
{
public:
  virtual ~SimLatencyPlugin()
  {
  }

  /*
  *  Called when the plugin is initialized
  */
  virtual bool initPlugin(const std::string& robot_namespace, ros::NodeHandle model_nh, int n_dof) = 0;

  /*
  *  Function implementing delay between Gazebo and ros control, called when RobotHWSimLatency::readSim() 
  * (the function updating the state of hardware interfaces with data from gazebo)
  *  is called.
  */
  virtual std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>>
  delayStates(ros::Time time, ros::Duration period, std::vector<double>& position, std::vector<double>& velocity,
              std::vector<double>& effort) = 0;


  /*
  *  Function implementing delay between Gazebo and ros control, called when RobotHWSimLatency::writeSim() 
  * (the function applying ros control effort, velocity or position commands on Gazebo joints)
  *  is called.
  *  Currently the return values of std::vector<double>& position, std::vector<double>& velocity,
  *  std::vector<double>& effort (the first three std::vector<double>) are not used.
  */
  virtual std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>,
                     std::vector<double>, std::vector<double>, std::vector<double>>
  delayCommands(ros::Time time, ros::Duration period, std::vector<double>& position, std::vector<double>& velocity,
                std::vector<double>& effort, std::vector<double>& position_command,
                std::vector<double>& velocity_command, std::vector<double>& effort_command) = 0;
};
}
