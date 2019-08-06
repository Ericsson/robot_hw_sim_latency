/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_ARIAC_TASK_MANAGER_PLUGIN_HH_
#define GAZEBO_ROS_ARIAC_TASK_MANAGER_PLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/SubmitTray.h>
#include <sdf/sdf.hh>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace gazebo
{
  // Forward declare private data class
  class ROSAriacTaskManagerPluginPrivate;

  /// \brief A plugin that orchestrates an ARIAC task. First of all, it loads a
  /// description of the orders. Here's an example:
  ///
  ///  <order>
  ///    <time>5.0</time>
  ///    <!-- 1st kit -->
  ///    <kit>
  ///      <object>
  ///        <type>coke_can</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>cordless_drill/</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>beer</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///    </kit>
  ///    <!-- 2nd kit -->
  ///    <kit>
  ///      <object>
  ///        <type>coke_can</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>cordless_drill/</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///      <object>
  ///        <type>beer</type>
  ///        <pose>-1 2.5 0.2 0 0 0</pose>
  ///      </object>
  ///    </kit>
  ///  </order>
  ///
  /// A task can have multiple orders. Each order has a time element. At that
  /// point in simulation time, the order will be notified to the team.
  /// An order is composed by a positive number of kits. A kit is composed by
  /// a positive number of objects. An object contains a type (e.g.: coke_can)
  /// and a pose. The pose is the target pose where the object should be placed
  /// on a destination tray.
  ///
  /// After loading the orders, the plugin will use a simple finite state machine
  /// to handle the different tasks to do.
  ///
  /// The initial state is called "init" and there's not much to do when the
  /// plugin is in this state. The state of the Gazebo task manager is
  /// periodically published on a ROS topic "gazebo_task/state". This topic can
  /// be changed using the SDF element <gazebo_task_state_topic>. The plugin is
  /// waiting for a ROS message on topic "team_task/state". This topic shows
  /// the state of the team performing the task. When the value of a message
  /// received is "ready", the plugin will transition its internal state towards
  /// "ready" too.
  ///
  /// The "ready" state is considered simulation time zero for notifying the
  /// orders to the teams. A <time>1.0</time> inside a <order> element will be
  /// notified 1 second after entering in the "ready" state. The order will be
  /// published using a ROS topic on the topic ${robot_namespace}/order or
  /// ${robot_namespace}/${orders_topic} if the parameter <orders_topic> is passed
  /// in to the plugin. Also, when the plugin is in this state, it will use the
  /// conveyor activation topic to communicate with the Population plugin and
  /// enable the population of the conveyor belt. The element
  /// <conveyor_activation_topic> should match the topic used in the
  /// <activation_topic> of the Population plugin. After enabling the conveyor
  /// belt, the state changes to "go".
  ///
  /// In "go" state, the plugin will be updating the function that processes
  /// the orders. This is essentially checking if it's time to announce a new
  /// order.
  ///
  /// In "finish" state there's nothing to do.
  class GAZEBO_VISIBLE ROSAriacTaskManagerPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: ROSAriacTaskManagerPlugin();

    /// \brief Destructor.
    public: virtual ~ROSAriacTaskManagerPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Update the plugin.
    protected: void OnUpdate();

    /// \brief Decide whether to announce a new order.
    protected: void ProcessOrdersToAnnounce();

    /// \brief Set the velocity of the conveyor belt.
    protected: void ControlConveyorBelt(double power);

    /// \brief Publish competition status.
    protected: void PublishStatus(const ros::TimerEvent&);

    /// \brief Start populating the conveyor belt.
    protected: void PopulateConveyorBelt();

    /// \brief Callback received when the team requests the competition start.
    public: bool HandleStartService(
      std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);

    /// \brief Callback received when the team requests the competition end.
    public: bool HandleEndService(
      std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);

    /// \brief Callback for when a tray is submitted for inspection.
    public: bool HandleSubmitTrayService(
      ros::ServiceEvent<osrf_gear::SubmitTray::Request, osrf_gear::SubmitTray::Response> & event);

    /// \brief Callback for when a query is made for material locations.
    public: bool HandleGetMaterialLocationsService(
      osrf_gear::GetMaterialLocations::Request & req, osrf_gear::GetMaterialLocations::Response & res);

    /// \brief Assign an order to be monitored by the scorer.
    protected: void AssignOrder(const ariac::Order & order);

    /// \brief Stop scoring the current order and assign the next order on stack.
    protected: void StopCurrentOrder();

    /// \brief Private data pointer.
    private: std::unique_ptr<ROSAriacTaskManagerPluginPrivate> dataPtr;
  };
}
#endif
