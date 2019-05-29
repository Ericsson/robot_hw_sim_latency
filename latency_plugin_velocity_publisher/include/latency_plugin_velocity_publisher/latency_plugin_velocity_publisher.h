#pragma once

#include <robot_hw_sim_latency/sim_latency_plugin.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <deque>
#include <bitset>
#include <mutex>
#include <tuple>

namespace latency_plugin_velocity_publisher
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


class VelocityPublisherPlugin : public robot_hw_sim_latency::SimLatencyPlugin
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
  const std::vector<std::string> joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                 "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  const double max_vel_change=25.0;

  void handleJointStateMessage(const sensor_msgs::JointStateConstPtr& m);

  int n_dof;
  std::vector<double> prev{6};
  std::mutex joint_state_mutex;
  std::vector<double> current_sim_joint_pos{6};
  std::vector<double> prev_sim_joint_pos{6};
  std::vector<double> current_sim_joint_vel{6};
  std_msgs::Float64MultiArray vel_msg;
  
  double prev_command[6];
  
  control_toolbox::Pid joint_pid[6];
  double joint_state_temp[6] = {0., 0., 0., 0., 0., 0.};
  
  ros::WallTime prev_time;

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber jointstate_sub;
  
};
}
