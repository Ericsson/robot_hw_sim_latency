from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from constants import ARM_JOINT_NAMES
import rospy
import angles
import tf2_ros
import tf as transf
import global_vars
from constants import *
import yaml
import math
import numpy
import transform

from math import cos
from copy import deepcopy
from trianglesolver import solve, degree

from transform import Transform
from enum import Enum


class PickPlaces(Enum):
    BELT    = "belt"
    ANY_BIN = "bin*"
    BIN1    = "bin1"
    BIN2    = "bin2"
    BIN3    = "bin3"
    BIN4    = "bin4"
    BIN5    = "bin5"
    BIN6    = "bin6"
    BIN7    = "bin7"
    BIN8    = "bin8"
    AGV1    = "agv1"
    AGV2    = "agv2"
    FAIL    = "fail"

def createJointTrajectory(angles, time):
    """
    Create a JointTrajectory object
    angles - list of joint angles
    time - time from start in seconds
    """
    global ARM_JOINT_NAMES
    msg = JointTrajectory()
    msg.joint_names = ARM_JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = angles
    point.time_from_start = rospy.Duration(time)
    msg.points = [point]
    return msg

def lookupPiece(part_name, destination, material):
    now = rospy.get_time()
    found_piece = False
    while True:
        for k, tf in global_vars.tf_transforms.items():
            if part_name in k and tf["valid"]:  # and k not in parts_used:
                # print part_name
                if destination == "belt":
                    # print (global_vars.last_belt_piece)
                    # print ("material: " + str(material))
                    value = global_vars.last_belt_piece.get(material)
                    if(value is not None and part_name in value):
                        part_name = global_vars.last_belt_piece[material]
                        tf["valid"] = False
                        found_piece = True
                        break
                else:
                    part_name = k
                    tf["valid"] = False
                    found_piece = True
                    break
        if found_piece or rospy.get_time()-now >= 20:
            break
    return part_name

def tf(toFrame, fromFrame):

    try:

        trans = global_vars.tfBuffer.lookup_transform(
            toFrame, fromFrame, rospy.Time.now(), rospy.Duration(10.0))

        # rospy.loginfo("lookup_transform took: %s"% (later-now) +"t2f result: %s" % trans)
        if(trans is None):
            rospy.logerr("[ActionCameraTF] Could no find TF") 
            return None

        transform = trans.transform
        return transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
        rospy.logerr("\n\n\nT2f exception toFrame: " + toFrame +
             "\n fromFrame: " + fromFrame + 
             "\n except: " + str(exc) + "\n\n\n" )


# Returns pos.translation[x, y, z] || pos.rotation[x, y, z, w]
def getUpperArmPose():
    transforms = global_vars.tf_manager.get_transform_list('upper_arm_link', 'world')
    r = transform.transform_list_to_world(transforms)
    if r:
        return r
    




def getForeArmPos():
    transforms = global_vars.tf_manager.get_transform_list('forearm_link', 'world')
    r = transform.transform_list_to_world(transforms)
    if r:
        return r


    


def getForeArmTipPos():
    """
        Probably broken
    """
    pos, angle = getForeArmPos()
    pos[2] = pos[2] + FORE_ARM
    # trans = transform.Transform(pos, angle)
    # retP = transform.transformDistance(trans, FORE_ARM)
    return pos, angle


def getVacuumGripperPos():
    transforms = global_vars.tf_manager.get_transform_list('tool0', 'world')
    r = transform.transform_list_to_world(transforms)
    if r:
        return r


def computeXDistance(pos1, pos2):
    return pos1[0] - pos2[0]


def computeZDistance(pos1, pos2):
    #If pos2 is lower than po1 the return is positive
    #If pos1 is lower than pos2 the return is negative
    #If they are at the same level, it is zero
    return pos1[2] - pos2[2]

def computeYDistance(pos1, pos2):
    return pos1[1] - pos2[1]


def computeYZDistance(pos1, pos2):
    dist = math.hypot(pos2[1] - pos1[1],
                      pos2[2] - pos1[2])
    return dist


def computeXDistance(pos1, pos2):
    return pos1[0] - pos2[0]


def computeZDistance(pos1, pos2):
    return pos1[2] - pos2[2]


def computeXZDistance(pos1, pos2):
    dist = math.hypot(pos2[0] - pos1[0],
                      pos2[2] - pos1[2])
    return dist


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    # Fixed. Otherwise it would consider as close x1 = -3.0 and x2 = 3.0 for
    # instance
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def iscloseError(a, b, error):
    #Fixed. Otherwise it would consider as close x1 = -3.0 and x2 = 3.0 for instance
    #return abs(a - b) <= 0.015
    return abs(a - b) <= error
    #return abs(angles.shortest_angular_distance(a,b)) <= error

def checkPartOnTray(vet1, vet2, op):
    diff0 = abs(vet1[0]) - abs(vet2[0])
    diff1 = abs(vet1[1]) - abs(vet2[1])

    if op == "ori":
        diff2 = abs(cos(vet1[2])) - abs(cos(vet2[2]))

    if op == "pos":
        print (diff0, diff1)
        if diff0 > 0.05 or diff1 > 0.05:
            return True
        else:
            return False
    else:
        print (vet1[2], vet2[2], diff2)
        if diff2 > 0.05:
            return True
        else:
            return False

def comparePosition(p1, p2, accError):
    goalToleranceRatio = 3
    l = min(len(p1), len(p2))
    ret = []
    eq = True
    errIdx = 0
    for i in range(l):
        ret.append(True)
        error = goalToleranceRatio * accError[errIdx]
        #rospy.logerr("tolerance: "+str(error))
        errIdx+=1
        #if not isclose(p1[i], p2[i], rel_tol=0.04, abs_tol=0.005):
        if not iscloseError(p1[i], p2[i], error):
            #rospy.logerr("ERROR: "+str(p1[i]) +" " + str(p2[i])+" index:"+str(i))
            ret[-1] = False
            eq = False


    return eq, ret

def calculate_order_position(order_pose, agv_id, incrementX=0, incrementY=0, incrementZ=0):
    tray_frame = TRAY_FRAME[agv_id]
    transform_part = Transform([order_pose.position.x, order_pose.position.y, order_pose.position.z], [
                               order_pose.orientation.x, order_pose.orientation.y, order_pose.orientation.z, order_pose.orientation.w])
    transform_agv = global_vars.tf_manager.get_transform('world', tray_frame)

    final_position, final_rotation = transform.transform_to_world(
        transform_part, transform_agv)

    # moving to the middle of the tray
    final_position[0] += TRAY_POSITIONS2[agv_id][0] + incrementX
    final_position[1] += TRAY_POSITIONS2[agv_id][1] + incrementY
    final_position[2] += TRAY_POSITIONS2[agv_id][2] + incrementZ

    final_rotation = transf.transformations.euler_from_quaternion([order_pose.orientation.x, order_pose.orientation.y, order_pose.orientation.z, order_pose.orientation.w])

    

    return final_position, final_rotation



