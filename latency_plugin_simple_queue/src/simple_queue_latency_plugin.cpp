/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#include <latency_plugin_simple_queue/simple_queue_latency_plugin.h>

#include <tuple>

namespace latency_plugin_simple_queue
{




bool SimpleQueueLatencyPlugin::initPlugin(const std::string& robot_namespace, ros::NodeHandle model_nh, int n_dof_)
{
	n_dof=n_dof_;
	queue_state =  std::deque<RobotData>(10, RobotData(n_dof_));
	queue_command =  std::deque<RobotData>(10, RobotData(n_dof_));
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>>
SimpleQueueLatencyPlugin::delayStates(ros::Time time, ros::Duration period, std::vector<double>& position,
                                     std::vector<double>& velocity, std::vector<double>& effort)
{
	queue_state.emplace_back(n_dof, time, period, position, velocity, effort);
	
	auto ret = queue_state.front();
	queue_state.pop_front();
	ROS_INFO_STREAM_THROTTLE_NAMED(1,"latency_plugin_simple_queue", "Current read latency: "<< time-ret.time);
	return std::make_tuple(time, period, ret.position, ret.velocity, ret.effort);
}

std::tuple<ros::Time, ros::Duration, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
           std::vector<double>, std::vector<double>>
SimpleQueueLatencyPlugin::delayCommands(ros::Time time, ros::Duration period, std::vector<double>& position,
                                       std::vector<double>& velocity, std::vector<double>& effort,
                                       std::vector<double>& position_command, std::vector<double>& velocity_command,
                                       std::vector<double>& effort_command)
{
	queue_command.emplace_back(n_dof, time, period, position, velocity, effort, position_command, velocity_command, effort_command);
	auto ret = queue_command.front();
	queue_command.pop_front();
	ROS_INFO_STREAM_THROTTLE_NAMED(1,"latency_plugin_simple_queue", "Current write latency: "<< time-ret.time);
	return std::make_tuple(time, period, position, velocity, effort, ret.position_command, ret.velocity_command, ret.effort_command);
}
}

PLUGINLIB_EXPORT_CLASS(latency_plugin_simple_queue::SimpleQueueLatencyPlugin, robot_hw_sim_latency::SimLatencyPlugin)
