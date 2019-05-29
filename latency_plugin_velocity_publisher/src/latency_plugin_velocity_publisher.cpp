#include <latency_plugin_velocity_publisher/latency_plugin_velocity_publisher.h>

namespace latency_plugin_velocity_publisher
{
bool VelocityPublisherPlugin::initPlugin(const std::string& robot_namespace, ros::NodeHandle model_nh, int n_dof_)
{
  n_dof = n_dof_;
  nh = ros::NodeHandle("/latency_plugin_velocity_publisher/");
  pub = nh.advertise<std_msgs::Float64MultiArray>("/latency_plugin_velocity_publisher/joint_speeds", 1);
  jointstate_sub = nh.subscribe<sensor_msgs::JointState>("/latency_plugin_velocity_publisher/joint_states", 1,
                                                         &VelocityPublisherPlugin::handleJointStateMessage, this);

  for (int i = 0; i < 6; i++) {
    // Pid(double p, double i, double d, double i_max, double i_min)
    joint_pid[i] = control_toolbox::Pid(1.0, 0.0, 0.0, 0.0, 0.0);
  }
  
  vel_msg.data.resize(6);

  ROS_INFO_STREAM_ONCE_NAMED("latency_plugin_velocity_publisher", "latency_plugin_velocity_publisher loaded");
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>>
VelocityPublisherPlugin::delayStates(ros::Time time, ros::Duration period, std::vector<double>& position,
                                     std::vector<double>& velocity, std::vector<double>& effort)
{
  std::lock_guard<std::mutex> guard(joint_state_mutex);
  current_sim_joint_pos = std::vector<double>(&position[0], &position[6]);
  return std::make_tuple(time, period, position, velocity, effort);
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
           std::vector<double>, std::vector<double>>
VelocityPublisherPlugin::delayCommands(ros::Time time, ros::Duration period, std::vector<double>& position,
                                       std::vector<double>& velocity, std::vector<double>& effort,
                                       std::vector<double>& position_command, std::vector<double>& velocity_command,
                                       std::vector<double>& effort_command)
{
  return std::make_tuple(time, period, position, velocity, effort, position_command, velocity_command, effort_command);
}

void VelocityPublisherPlugin::handleJointStateMessage(const sensor_msgs::JointStateConstPtr& message)
{
  // If the message doesn't contain at least 6 joints, return.
  // So we are not spammed with warnings.
  if (message->name.size() < 6) {
    return;
  }

  // Because we can not be sure if the message contains the joints in order,
  // we have to find the index of the joint, and update the proper joint.
  for (std::size_t i = 0; i < joint_names.size(); i++) {
    auto const& joint_name = joint_names[i];
    auto pos = std::find(message->name.begin(), message->name.end(), "realur/" + joint_name);
    if (pos != message->name.end()) {
      auto index = pos - message->name.begin();

      joint_state_temp[i] = message->position[index];
    } else {
      ROS_WARN_STREAM("Did not find joint in joint state message: " << joint_name);
      return;
    }
  }

  double error = 0;

  ros::WallTime now = ros::WallTime::now();
  ros::Duration period{(now - prev_time).toSec()};
  std::unique_lock<std::mutex> lock(joint_state_mutex);
  for (int i = 0; i < 6; i++) {
    error = current_sim_joint_pos[i] - joint_state_temp[i];
    if (std::abs(error) < 0.0005) {
      error = 0;
    }
    // ROS_INFO_STREAM("POS error "<<i<<" :"<<error);
    // ROS_INFO_STREAM(i<<":"<<error<<"="<<current_sim_joint_pos[i]<<"-"<<joint_state_temp[i];);
    double vel = (current_sim_joint_pos[i] - prev_sim_joint_pos[i]) / period.toSec();
    double command = vel + joint_pid[i].computeCommand(error, period);
    if (command > max_vel_change) {
      command *= max_vel_change / command;
    }
    vel_msg.data[i] = command;  //(command+prev_command[i])/2;
    prev_command[i] = command;

    //ROS_INFO_STREAM("POS command " << i << ": " << command);
  }
  lock.unlock();

  pub.publish(vel_msg);
  //ROS_INFO_STREAM(vel_msg);
  prev_time = now;
  prev_sim_joint_pos = current_sim_joint_pos;
}
}

PLUGINLIB_EXPORT_CLASS(latency_plugin_velocity_publisher::VelocityPublisherPlugin,
                       robot_hw_sim_latency::SimLatencyPlugin)
