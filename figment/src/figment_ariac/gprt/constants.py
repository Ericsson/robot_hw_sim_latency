import os
import rospy
from rospkg import RosPack
rp = RosPack()

STATIC_POSITIONS = {
    "bin5"	: [2.8, -1.1661, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "bin6"	: [2.8, -0.3711, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "bin7"	: [2.8, 0.3939, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "bin8"	: [2.8, 1.1589, -1.308996939, 3.1415926536, 3.1415926536, -1.5707963268, 0],
    "agv1": [2.8, 2.1, -1.308996939, 1.57, 3.1415926536, -1.5707963268, 0],
    "agv2": [2.8, -2.1, -1.308996939, 4.71, 3.1415926536, -1.5707963268, 0],
    "disBelAgv2": [2.8, -1.70, -0.88, 5.91, 3.77, -1.57, 0.0],
    "disBelAgv2Back": [2.80, -2.10, -1.57, 5.91, 3.39, -1.57, 0.0],
    "disBelAgv2Open": [1.79, -2.10, -1.01, 6.03, 3.90, -1.57, 0.0],
    "disBelAgv1":     [2.8, 0.80, -1.63, 0.13, 3.52, -1.57, 0.0],
    "disBelAgv1Open": [1.57, 0.80, -0.75, 0.0, 3.77, -1.57, 0.0],
    "disBelAgv1Back": [1.70, 0.80, -0.75, 0.0, 3.77, -1.57, 0.0],
    "belt": [1.89, 2.1, -1.13, 0, 4.52, -1.57, 0.0],
    "beltStart": [2.82, 2.1, -1.57, 0, 3.39, -1.57, 0.0],
    "initial_position": [2.51, 0.0, -1.13, 3.14, 3.14, -1.51, 0.0],
    "rest_position" : [2.75, 0.0, -1.308996939, 3.1415926536, 3.27, -1.5707963268, 0.0] 
}

BINS_CAMERA = {
    "bin5" : "logical_camera_bin_5_6",
    "bin6"  : ["logical_camera_bin_5_6", "logical_camera_bin_6_7"],
    "bin7"  : ["logical_camera_bin_6_7", "logical_camera_bin_7_8"],
    "bin8"  : "logical_camera_bin_7_8"
}


AGVS_CAMERA = {
    "agv1": "logical_camera_agv_1",
    "agv2": "logical_camera_agv_2",
}

BELT_CAMERA = {
    "belt"  : "logical_camera_belt_1"
}

ORIGN_CAMERA = {}
ORIGN_CAMERA.update(BINS_CAMERA)
ORIGN_CAMERA.update(BELT_CAMERA)

ALL_CAMERA = {}
ALL_CAMERA.update(ORIGN_CAMERA)
ALL_CAMERA.update(AGVS_CAMERA)


TRAY_FRAME = {
    1 : "agv1_load_point_frame",
    2 : "agv2_load_point_frame"
}

TRAY_POSITIONS = {
    1: [0.3002, 3.15, 0.77],
    2: [0.3, -3.15, 0.77]
}

TRAY_POSITIONS2 = {
    1: [0, -0.15, 0.03],
    2: [0, 0.15, 0.03]
}

# ARM_JOINT ={
#     'elbow_joint':0,
#     'linear_arm_actuator_joint':1,
#     'shoulder_lift_joint':2,
#     'shoulder_pan_joint':3,
#     'wrist_1_joint':4,
#     'wrist_2_joint':5,
#     'wrist_3_joint':6,    

# }

ARM_JOINT_NAMES = [
    'elbow_joint',
    'linear_arm_actuator_joint',
    'shoulder_lift_joint',
    'shoulder_pan_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

X_BASE = 0.3
Y_BASE = 2.1
Z_BASE = 0.9999

# BASE_UPPER_DIST = 0.2208
BASE_UPPER_DIST = 0.18

DX_BASE = 0.13235
#DX_BASE = 0.13
DY_BASE = 2.14972
DZ_BASE = 0.9999

GRIPPER = 0.01 + 0.005415
WRIST_1_2 =  0.0922

H_BASE = 0.128
H_WRIST = 0.1157

UP_ARM = 0.6127
FORE_ARM = 0.5716

WRIST_LENGTH = 0.1639

LOGICAL_CAMERA_BIN_7_8_Y = 0.610722
LOGICAL_CAMERA_BIN_5_6_Y = -0.914553

OBJECT_HEIGHT = {
    "PISTON_ROD_PART": 0.005,
    "GEAR_PART": 0.006,
    "PULLEY_PART": 0.0719, 
    "PULLEY_PART_TURN": 0.0719,
    "GASKET_PART": 0.0223999,
    "DISK_PART": 0.0248
}

this_dir = os.path.abspath(os.path.dirname(__file__))
template_files = [
    os.path.join(rp.get_path('figment_ariac'), 'config', 'figment_gear_conf.yaml')
]

KIT_TIMEOUT = rospy.Duration.from_sec(400) #ODO check what time would be best
COMP_TIMEOUT = rospy.Duration.from_sec(480) #ODO check what time would be best

PLAN_STEP_CNT = rospy.Duration.from_sec(5) #ODO check what time would be best
MAX_PLAN_STEP_CNT = 10
MAX_PLAN_ERR = 6

