#!/usr/bin/env python
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import time

import rospy

from osrf_gear.msg import Order
from osrf_gear.msg import VacuumGripperState
from osrf_gear.srv import AGVControl
from osrf_gear.srv import VacuumGripperControl
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def start_competition():
    rospy.loginfo("Waiting for competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")

    try:
        start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        response = start()
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to start the competition: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to start the competition: %s" % response)
    else:
        rospy.loginfo("Competition started!")
    return response.success


def control_gripper(enabled):
    rospy.loginfo("Waiting for gripper control to be ready...")
    rospy.wait_for_service('/ariac/gripper/control')
    rospy.loginfo("Gripper control is now ready.")
    rospy.loginfo("Requesting gripper control...")

    try:
        gripper_control = rospy.ServiceProxy('/ariac/gripper/control', VacuumGripperControl)
        response = gripper_control(enabled)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to control the gripper: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to control the gripper: %s" % response)
    else:
        rospy.loginfo("Gripper controlled successfully")
    return response.success


def control_agv(index, kit_type):
    rospy.loginfo("Waiting for AGV control to be ready...")
    name = '/ariac/agv' + str(index)
    rospy.wait_for_service(name)
    rospy.loginfo("AGV control is now ready.")
    rospy.loginfo("Requesting AGV control...")

    try:
        agv_control = rospy.ServiceProxy(name, AGVControl)
        response = agv_control(kit_type)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to control the AGV: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to control the AGV: %s" % response)
    else:
        rospy.loginfo("AGV controlled successfully")
    return response.success


class MyCompetitionClass:
    def __init__(self):
        self.joint_trajectory_publisher = \
            rospy.Publisher("/ariac/arm/command", JointTrajectory, queue_size=10)
        self.current_comp_state = None
        self.received_orders = []
        self.current_joint_state = None
        self.current_gripper_state = None
        self.last_joint_state_print = time.time()
        self.last_gripper_state_print = time.time()
        self.has_been_zeroed = False
        self.arm_joint_names = [
            'elbow_joint',
            'linear_arm_actuator_joint',
            'shoulder_lift_joint',
            'shoulder_pan_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            rospy.loginfo("Competition state: " + str(msg.data))
        self.current_comp_state = msg.data

    def order_callback(self, msg):
        rospy.loginfo("Received order:\n" + str(msg))
        self.received_orders.append(msg)

    def joint_state_callback(self, msg):
        if time.time() - self.last_joint_state_print >= 10:
            rospy.loginfo("Current Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_joint_state_print = time.time()
        self.current_joint_state = msg

    def gripper_state_callback(self, msg):
        if time.time() - self.last_gripper_state_print >= 10:
            rospy.loginfo("Current gripper state (throttled to 0.1 Hz):\n" + str(msg))
            self.last_gripper_state_print = time.time()
        self.current_gripper_state = msg

    def send_arm_to_state(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]
        rospy.loginfo("Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)


def connect_callbacks(comp_class):
    comp_state_sub = rospy.Subscriber(
        "/ariac/competition_state", String, comp_class.comp_state_callback)
    order_sub = rospy.Subscriber("/ariac/orders", Order, comp_class.order_callback)
    joint_state_sub = rospy.Subscriber(
        "/ariac/joint_states", JointState, comp_class.joint_state_callback)
    gripper_state_sub = rospy.Subscriber(
        "/ariac/gripper/state", VacuumGripperState, comp_class.gripper_state_callback)
