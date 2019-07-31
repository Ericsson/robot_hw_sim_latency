#include <latency_plugin_simple_queue/simple_queue_latency_plugin.h>

namespace latency_plugin_simple_queue
{
bool SimpleQueueLatencyPlugin::initPlugin(const std::string& robot_namespace, ros::NodeHandle model_nh, int n_dof_)
{
  n_dof = n_dof_;
  nh = ros::NodeHandle("latency_plugin_simple_queue/");
  queue_length = nh.param<int>("latency_plugin_simple_queue/queue_length", 0);
  low_latency_queue_length = 1;
  ros::AsyncSpinner spinner(1);

  low_latency = false;
  prev_low_latency = false;

  sub = nh.subscribe("latency_plugin_simple_queue/priority", 1, &SimpleQueueLatencyPlugin::queue_length_callback, this);
  //~ sub = nh.subscribe("/figment/trajectory/precision", 1, &SimpleQueueLatencyPlugin::queue_length_callback, this);

  no_delay = queue_length == 0 ? true : false;
  if (!no_delay) {
    queue_state = std::deque<RobotData>(queue_length, RobotData(n_dof_));
    queue_command = std::deque<RobotData>(queue_length, RobotData(n_dof_));
  }

  spinner.start();

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

  if (latency_change_state.test(StateQueueSetToLow)) {
    latency_change_state.reset(StateQueueSetToLow);
    queue_state.erase(queue_state.begin(), queue_state.begin() + (queue_state.size() - low_latency_queue_length));
    
    ROS_INFO_STREAM_NAMED("latency_plugin_simple_queue", "State queue changed to size " << queue_state.size());
  }
  if (latency_change_state.test(StateQueueSetToHigh)) {
    latency_change_state.reset(StateQueueSetToHigh);
    queue_state.resize(queue_length, RobotData(n_dof, time, period, position, velocity, effort));

    ROS_INFO_STREAM_NAMED("latency_plugin_simple_queue", "State queue changed to size " << queue_state.size());
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
  if (latency_change_state.test(CommandQueueSetToLow)) {
    latency_change_state.reset(CommandQueueSetToLow);
    queue_command.erase(queue_command.begin(),
                        queue_command.begin() + (queue_command.size() - low_latency_queue_length));

    ROS_INFO_STREAM_NAMED("latency_plugin_simple_queue", "Command queue changed to size " << queue_command.size());
  }
  if (latency_change_state.test(CommandQueueSetToHigh)) {
    latency_change_state.reset(CommandQueueSetToHigh);
    queue_command.resize(queue_length, RobotData(n_dof, time, period, position, velocity, effort, position_command,
                                                 velocity_command, effort_command));

    ROS_INFO_STREAM_NAMED("latency_plugin_simple_queue", "Command queue changed to size " << queue_command.size());
  }

  queue_command.emplace_back(n_dof, time, period, position, velocity, effort, position_command, velocity_command,
                             effort_command);
  auto ret = queue_command.front();
  queue_command.pop_front();

  ROS_INFO_STREAM_THROTTLE_NAMED(1, "latency_plugin_simple_queue", "Current write latency: " << time - ret.time);

  return std::make_tuple(time, period, position, velocity, effort, ret.position_command, ret.velocity_command,
                         ret.effort_command);
}

void SimpleQueueLatencyPlugin::queue_length_callback(const std_msgs::StringConstPtr& priom)
{
  prev_low_latency = low_latency;
  std::string prio = priom->data;
  if (prio == "Low") {
    low_latency = false;
  } else {
    low_latency = true;
  }

  if (!prev_low_latency && low_latency) {
    latency_change_state.reset();
    latency_change_state.set(CommandQueueSetToLow);
    latency_change_state.set(StateQueueSetToLow);
  }

  if (prev_low_latency && !low_latency) {
    latency_change_state.reset();
    latency_change_state.set(CommandQueueSetToHigh);
    latency_change_state.set(StateQueueSetToHigh);
  }
  ROS_INFO_STREAM_NAMED("latency_plugin_simple_queue", "Changed to latency mode " << (low_latency ? "Low" : "High"));
}
}

PLUGINLIB_EXPORT_CLASS(latency_plugin_simple_queue::SimpleQueueLatencyPlugin, robot_hw_sim_latency::SimLatencyPlugin)
