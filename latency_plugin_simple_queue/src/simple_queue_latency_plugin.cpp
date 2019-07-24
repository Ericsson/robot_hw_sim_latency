#include <latency_plugin_simple_queue/simple_queue_latency_plugin.h>

#include <tuple>

namespace latency_plugin_simple_queue
{
bool SimpleQueueLatencyPlugin::initPlugin(const std::string& robot_namespace, ros::NodeHandle model_nh, int n_dof_)
{
  n_dof = n_dof_;
  queue_length = model_nh.param<int>("latency_plugin_simple_queue/queue_length", 0);
  no_delay = queue_length == 0 ? true : false;
  if (!no_delay) {
    queue_state = std::deque<RobotData>(queue_length, RobotData(n_dof_));
    queue_command = std::deque<RobotData>(queue_length, RobotData(n_dof_));
  }
  ROS_INFO_STREAM_ONCE_NAMED("latency_plugin_simple_queue",
                             "latency_plugin_simple_queue loaded Queue length: " << queue_length);
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>>
SimpleQueueLatencyPlugin::delayStates(ros::Time time, ros::Duration period, std::vector<double>& position,
                                      std::vector<double>& velocity, std::vector<double>& effort)
{
  if (no_delay) {
    return std::make_tuple(time, period, position, velocity, effort);
  }
  queue_state.emplace_back(n_dof, time, period, position, velocity, effort);

  auto ret = queue_state.front();
  queue_state.pop_front();
  ROS_INFO_STREAM_THROTTLE_NAMED(1, "latency_plugin_simple_queue", "Current read latency: " << time - ret.time);
  return std::make_tuple(time, period, ret.position, ret.velocity, ret.effort);
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
           std::vector<double>, std::vector<double>>
SimpleQueueLatencyPlugin::delayCommands(ros::Time time, ros::Duration period, std::vector<double>& position,
                                        std::vector<double>& velocity, std::vector<double>& effort,
                                        std::vector<double>& position_command, std::vector<double>& velocity_command,
                                        std::vector<double>& effort_command)
{
  if (no_delay) {
    return std::make_tuple(time, period, position, velocity, effort, position_command, velocity_command,
                           effort_command);
  }
  queue_command.emplace_back(n_dof, time, period, position, velocity, effort, position_command, velocity_command,
                             effort_command);
  auto ret = queue_command.front();
  queue_command.pop_front();
  ROS_INFO_STREAM_THROTTLE_NAMED(1, "latency_plugin_simple_queue", "Current write latency: " << time - ret.time);
  return std::make_tuple(time, period, position, velocity, effort, ret.position_command, ret.velocity_command,
                         ret.effort_command);
}
}

PLUGINLIB_EXPORT_CLASS(latency_plugin_simple_queue::SimpleQueueLatencyPlugin, robot_hw_sim_latency::SimLatencyPlugin)
