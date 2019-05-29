#pragma once

#include <ros/ros.h>
#include <robot_hw_sim_latency/sim_latency_plugin.h>
#include <tuple>

namespace robot_hw_sim_latency
{
class DefaultSimLatencyPlugin: public SimLatencyPlugin
{
public:

  virtual bool initPlugin(const std::string& robot_namespace, ros::NodeHandle model_nh, int n_dof);

  virtual std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>>
  delayStates(ros::Time time, ros::Duration period, std::vector<double>& position, std::vector<double>& velocity,
              std::vector<double>& effort);

  virtual std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>,
                     std::vector<double>, std::vector<double>, std::vector<double>>
  delayCommands(ros::Time time, ros::Duration period, std::vector<double>& position, std::vector<double>& velocity,
                std::vector<double>& effort, std::vector<double>& position_command,
                std::vector<double>& velocity_command, std::vector<double>& effort_command);
};
}
