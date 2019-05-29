#pragma once

#include <robot_hw_sim_latency/sim_latency_plugin.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <deque>
#include <bitset>
#include <tuple>

namespace latency_plugin_simple_queue
{
struct RobotData
{
  ros::Time time;
  ros::Duration period;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> position_command;
  std::vector<double> velocity_command;
  std::vector<double> effort_command;
  
  RobotData(int n_dof, ros::Time time_, ros::Duration period_, std::vector<double> position_,
            std::vector<double> velocity_, std::vector<double> effort_, std::vector<double> position_command_,
            std::vector<double> velocity_command_, std::vector<double> effort_command_)
    : time(time_)
    , period(period_)
    , position(position_)
    , velocity(velocity_)
    , effort(effort_)
    , position_command(position_command_)
    , velocity_command(velocity_command_)
    , effort_command(effort_command_)
  {
  }
  RobotData(int n_dof, ros::Time time_, ros::Duration period_, std::vector<double> position_,
            std::vector<double> velocity_, std::vector<double> effort_)
    : time(time_)
    , period(period_)
    , position(position_)
    , velocity(velocity_)
    , effort(effort_)
    , position_command(n_dof)
    , velocity_command(n_dof)
    , effort_command(n_dof)
  {
  }
  RobotData(int n_dof)
    : time()
    , period()
    , position(n_dof)
    , velocity(n_dof)
    , effort(n_dof)
    , position_command(n_dof)
    , velocity_command(n_dof)
    , effort_command(n_dof)
  {
  }
};

enum LatencyChangeState{
  CommandQueueSetToLow,
  CommandQueueSetToHigh,
  StateQueueSetToLow,
  StateQueueSetToHigh,
};

class SimpleQueueLatencyPlugin : public robot_hw_sim_latency::SimLatencyPlugin
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

private:
  void queue_length_callback(const std_msgs::String::ConstPtr&);
  std::deque<RobotData> queue_state;
  std::deque<RobotData> queue_command;

  int n_dof;
  int queue_length;
  int low_latency_queue_length;
  bool no_delay;
  bool low_latency;
  bool prev_low_latency;
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  
  std::bitset<4> latency_change_state;
  
};
}
