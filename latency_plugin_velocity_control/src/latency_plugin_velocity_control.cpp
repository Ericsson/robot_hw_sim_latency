#include <latency_plugin_velocity_control/latency_plugin_velocity_control.h>

namespace latency_plugin_velocity_control
{
bool VelocityControlPlugin::initPlugin(const std::string& robot_namespace, ros::NodeHandle model_nh, int n_dof_)
{
  n_dof = n_dof_;
  nh = ros::NodeHandle("/latency_plugin_velocity_control/");
  pub = nh.advertise<std_msgs::Float64MultiArray>("/latency_plugin_velocity_control/joint_speeds", 1);
  jointstate_sub = nh.subscribe<sensor_msgs::JointState>("/latency_plugin_velocity_control/joint_states", 1,
                                                         &VelocityControlPlugin::handleJointStateMessage, this);

  //const double params[2][6] = {{0.3, 0.3, 0.3, 0.4, 0.4, 0.4}, {0.3, 0.3, 0.3, 0.4, 0.4, 0.4}};
  const double params[6] = {0.1, 0.1, 0.1, 0.2, 0.2, 0.2};
  for (int i = 0; i < 6; i++) {
    joint_ext_pid[i] = control_toolbox::Pid(params[i], 0.00, 0.00, 5.0, -5.0);
  }

  vel_msg.data.resize(6);
  prev.resize(6);
  current_sim_joint_pos.resize(6);
  current_sim_joint_vel.resize(6);

  prev_sim_joint_pos.resize(6);

  current_command_joint_pos.resize(6);
  current_command_joint_vel.resize(6);

  command_sim_joint_vel.resize(6);

  current_joint_state_pos.resize(6);
  current_joint_state_vel.resize(6);
  
  max_joint_pos_diff.resize(6);

  ROS_INFO_STREAM_ONCE_NAMED("latency_plugin_velocity_control", "latency_plugin_velocity_control loaded");
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>>
VelocityControlPlugin::delayStates(ros::Time time, ros::Duration period, std::vector<double>& position,
                                   std::vector<double>& velocity, std::vector<double>& effort)
{
  //~ for (int i = 0; i < n_dof; i++) {
    //~ ROS_WARN_STREAM(i<<"-"<<position[i]<<" "<<velocity[i]);
  //~ }
  //~ std::lock_guard<std::mutex> guard(joint_state_sim_mutex);
  //~ current_sim_joint_pos = std::vector<double>(&position[0], &position[6]);
  return std::make_tuple(time, period, position, velocity, effort);
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
           std::vector<double>, std::vector<double>>
VelocityControlPlugin::delayCommands(ros::Time time, ros::Duration period, std::vector<double>& position,
                                     std::vector<double>& velocity, std::vector<double>& effort,
                                     std::vector<double>& position_command, std::vector<double>& velocity_command,
                                     std::vector<double>& effort_command)
{
  // constexpr std::array<double,6> test_pos = {3.14,-1,1.51,3.77,-1.51,0.0};
  //~ for (int i = 0; i < n_dof; i++) {
    //~ ROS_WARN_STREAM(i<<" "<<position_command[i]<<" "<<velocity_command[i]);
  //~ }
  if (started) {
    std::unique_lock<std::mutex> lock(joint_comm_mutex);

    current_command_joint_pos = std::vector<double>(&position_command[0], &position_command[6]);
    current_command_joint_vel = std::vector<double>(&velocity_command[0], &velocity_command[6]);

    lock.unlock();

    // Command simulation from ext. joint state
    std::unique_lock<std::mutex> lock2(joint_comm_from_state_mutex);
    for (int i = 0; i < 6; i++) {

      double diff = current_joint_state_pos[i]-position[i];
      if (diff>max_joint_pos_diff[i]){
        max_joint_pos_diff[i] = diff;
        //ROS_WARN_STREAM(i<<","<<diff);
      }

      velocity_command[i] = current_joint_state_vel[i];
      position_command[i] = current_joint_state_pos[i] + velocity_command[i] * period.toSec() * accumulator;
      accumulator++;
    }
    // velocity_command[6] = velocity_command[6];
    // position_command[6] = position_command[6];
    lock2.unlock();
  }

  return std::make_tuple(time, period, position, velocity, effort, position_command, velocity_command, effort_command);
}

void VelocityControlPlugin::handleJointStateMessage(const sensor_msgs::JointStateConstPtr& message)
{
  // ROS_WARN_STREAM(*message);
  // If the message doesn't contain at least 6 joints, return.
  // So we are not spammed with warnings.
  if (message->name.size() < 6) {
    return;
  }

  // Because we can not be sure if the message contains the joints in order,
  // we have to find the index of the joint, and update the proper joint.
  std::unique_lock<std::mutex> lock2(joint_comm_from_state_mutex);
  for (std::size_t i = 0; i < joint_names.size(); i++) {
    auto const& joint_name = joint_names[i];
    auto pos = std::find(message->name.begin(), message->name.end(), "realur/" + joint_name);
    if (pos != message->name.end()) {
      auto index = pos - message->name.begin();

      current_joint_state_pos[i] = message->position[index];
      current_joint_state_vel[i] = message->velocity[index];
      
    } else {
      ROS_WARN_STREAM("Did not find joint in joint state message: " << joint_name);
      return;
    }
  }
  accumulator = 0;
  lock2.unlock();

  if (!started && startup_counter < 15) {
    startup_counter++;
    prev_time = ros::WallTime::now();
    return;
  }
  started = true;

  double error = 0;

  ros::WallTime now = ros::WallTime::now();
  ros::Duration period{(now - prev_time).toSec()};

  std::ostringstream debug_str;
  // Command external from sim command(delayCommands)
  std::unique_lock<std::mutex> lock(joint_comm_mutex);
  for (int i = 0; i < 6; i++) {
    //        target                       - current
    error = (current_command_joint_pos[i]) - current_joint_state_pos[i];
    if (std::abs(error) < 0.0005) {
      error = 0;
    }
    error /= period.toSec();
    double command = current_command_joint_vel[i] *0.1 + joint_ext_pid[i].computeCommand(error, period);


    //debug_str << current_command_joint_vel[i] << ',' << command << '|';
    if (fabs(command) > max_vel_change) {
      command *= max_vel_change / fabs(command);
    }
    vel_msg.data[i] = command;
    current_command_joint_vel[i] = 0;
  }
  // ROS_WARN_STREAM(debug_str.str());
  prev_time = now;
  prev_period = period;
  lock.unlock();

  // ROS_INFO_STREAM(vel_msg);
  pub.publish(vel_msg);
}
}

PLUGINLIB_EXPORT_CLASS(latency_plugin_velocity_control::VelocityControlPlugin, robot_hw_sim_latency::SimLatencyPlugin)
