#!/usr/bin/env python
from __future__ import print_function

import time
import tf2_ros
import tf as transf
import rospy
import yaml
import math
import utils
import global_vars
from tf_manager import *



from constants import *
from osrf_gear.msg import Order, VacuumGripperState, LogicalCameraImage, ConveyorBeltState
from osrf_gear.srv import VacuumGripperControl, GetMaterialLocations, ConveyorBeltControl, AGVControl
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, TransformStamped

import order_utils

import scheduler

class Competition:

    def __init__(self):
        self.current_comp_state = None
        self.received_orders = []
        self.current_gripper_state = None
        self.last_joint_state_print = time.time()
        self.last_gripper_state_print = time.time()
        self.has_been_zeroed = False

        # store the orders from the ariac competition
        self.orders = []
        # store the actions which the robot will execute
        self.actions_tray1 = []
        self.actions_tray2 = []
        # True when activated
        self.beltState = True
        self.scheduler = scheduler.Scheduler(self)


    def start_plan_and_execute(self):
        rospy.loginfo("[Competition] Starting plan and execute...")
        self.scheduler.execute()

    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            rospy.loginfo("[Competition] State CallBack - comp state: " + str(msg.data))
        self.current_comp_state = msg.data
        if msg.data == "done":
            self.scheduler.setFinished(True)

    def speed_modifier_callback(self, msg):
        global_vars.speed_modifier = msg.data

    def agv_1_state_callback(sel, ariac_agv_state_msg):
        global_vars.agv1_status = ariac_agv_state_msg.data  
        
    def agv_2_state_callback(sel, ariac_agv_state_msg):
        global_vars.agv2_status = ariac_agv_state_msg.data   

    def order_callback(self, ariac_order_msg):
    	order = order_utils.Order(ariac_order_msg)    	
        rospy.loginfo('New order received. {}'.format(order.get_full_repr()))
        self.scheduler.append_order(order)
     
    def joint_state_callback(self, msg):
        if time.time() - self.last_joint_state_print >= 10:
            self.last_joint_state_print = time.time()
        global_vars.current_joint_state = msg

    def gripper_state_callback(self, msg):

        if time.time() - self.last_gripper_state_print >= 10:
            self.last_gripper_state_print = time.time()
        self.current_gripper_state = msg
        global_vars.gripper_state = self.current_gripper_state

    def quality_control_sensor_1_callback(self, msg):
        pose = msg.pose  # camera position, DO NOT KNOW IF IT IS NEEDED
        global_vars.faulty_sensor1 = msg.models

    def quality_control_sensor_2_callback(self, msg):
        pose = msg.pose  # camera position, DO NOT KNOW IF IT IS NEEDED
        global_vars.faulty_sensor2 = msg.models


def camera_callback(msg, objs):
    
    camera_id, gprt = objs
    objects_hash = str(hash("".join([obj.type for obj in msg.models])))
    if global_vars.logical_cameras[camera_id]["hash"] != objects_hash:
        global_vars.logical_cameras[camera_id]["hash"] = objects_hash
        if camera_id == ORIGN_CAMERA["belt"]:
            if len(msg.models) > 0:
                for model in msg.models:
                    if "part" in model.type:
                        gprt.beltState = False
                    else:
                        gprt.beltState = True
                        
        global_vars.logical_cameras[camera_id]["models"] = msg.models
        global_vars.logical_cameras[camera_id]["camera_position"] = msg.pose


def setup_sensors(gprt):
    
    stream = file(template_files[0], 'r')
    config_file = yaml.load(stream)
    sensors = config_file['sensors']
    for k, sensor in sensors.items():
        if sensor['type'] == "break_beam":
            pass
        elif sensor['type'] == "proximity_sensor":
            pass
        elif sensor['type'] == "laser_profiler":
            pass
        elif sensor['type'] == "logical_camera":
            subscribe_path = "/ariac/" + str(k)
            rospy.loginfo("camera " + subscribe_path + " added.")
            global_vars.logical_cameras[k] = {
                "hash": 0,
                "models": [],
                "camera_position": None
            }
            camera_sub = rospy.Subscriber(
                subscribe_path, LogicalCameraImage, callback=camera_callback, callback_args=(k, gprt))

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

def connect_callbacks(comp_class):
    rospy.loginfo("[connect_callbacks] setting comp_state_callback")
    comp_state_sub = rospy.Subscriber(
        "/ariac/competition_state", String, comp_class.comp_state_callback)
    rospy.loginfo("[connect_callbacks] comp_state_callback OK")
    rospy.loginfo("[connect_callbacks] setting order_callback")
    order_sub = rospy.Subscriber(
        "/ariac/orders", Order, comp_class.order_callback)
    rospy.loginfo("[connect_callbacks] order_callback OK")
    rospy.loginfo("[connect_callbacks] setting agv_1_state_callback")
    agv_1_state_sub = rospy.Subscriber(
        "/ariac/agv1/state", String, comp_class.agv_1_state_callback)
    rospy.loginfo("[connect_callbacks] agv_1_state_callback OK")
    rospy.loginfo("[connect_callbacks] setting agv_1_state_callback")
    agv_2_state_sub = rospy.Subscriber(
        "/ariac/agv2/state", String, comp_class.agv_2_state_callback)
    rospy.loginfo("[connect_callbacks] agv_2_state_callback OK")  
    rospy.loginfo("[connect_callbacks] setting joint_state_callback")
    joint_state_sub = rospy.Subscriber(
        "/ariac/arm1/joint_states", JointState, comp_class.joint_state_callback)
    rospy.loginfo("[connect_callbacks] joint_state_callback OK")
    rospy.loginfo("[connect_callbacks] setting gripper_state_callback")
    gripper_state_sub = rospy.Subscriber(
        "/ariac/arm1/gripper/state", VacuumGripperState, comp_class.gripper_state_callback)
    rospy.loginfo("[connect_callbacks] gripper_state_callback OK")
    rospy.loginfo("[connect_callbacks] setting quality_control_sensor_1_callback")
    quality_control_sensor_1 = rospy.Subscriber(
        "/ariac/quality_control_sensor_1", LogicalCameraImage, comp_class.quality_control_sensor_1_callback)
    rospy.loginfo("[connect_callbacks] quality_control_sensor_1_callback OK")
    rospy.loginfo("[connect_callbacks] setting quality_control_sensor_2_callback")
    quality_control_sensor_2 = rospy.Subscriber(
        "/ariac/quality_control_sensor_2", LogicalCameraImage, comp_class.quality_control_sensor_2_callback)
    rospy.loginfo("[connect_callbacks] quality_control_sensor_2_callback OK")
    rospy.loginfo("[connect_callbacks] setting tf_callback")
    tf_sub = rospy.Subscriber(
        "/tf", TFMessage, callback=global_vars.tf_manager.tf_callback)
    rospy.loginfo("[connect_callbacks] tf_callback OK")
    tf_static_sub = rospy.Subscriber(
        "/tf_static", TFMessage, callback=global_vars.tf_manager.tf_static_callback)
    rospy.loginfo("[connect_callbacks] tf_static_callback OK")
    rospy.loginfo("[connect_callbacks] setting speed_modifier_callback")
    agv_1_state_sub = rospy.Subscriber(
        "/figment/trajectory/speed", Float32, comp_class.speed_modifier_callback)
    rospy.loginfo("[connect_callbacks] speed_modifier_callback OK")
    
def init_global_vars(comp_class):
    global_vars.tf_manager = TfManager()
    belt_camera_frame_id = BELT_CAMERA['belt'] + '_frame'
    global_vars.tf_manager.add_to_buffer(belt_camera_frame_id)