"""
    All commands that the arm will execute in the simulated world will be summarized here.


"""


import rospy
import utils
import global_vars
import math
import sys
import std_msgs
import inspect
import operator
import subprocess
import os

from constants import *
from trajectory_msgs.msg import JointTrajectory
from trianglesolver import solve, degree
from copy import deepcopy

from ik_solution import solverBin, solverBelt, depositOnTray2, depositOnTray1
import gripper_actions

import global_vars


class SolverType:
    BIN, AGV1, AGV2, BELT = range(4)


def init():
    """
    Initialize the global vars needed for execution
    """
    global joint_trajectory_publisher
    joint_trajectory_publisher = rospy.Publisher(
        "/ariac/arm/command", JointTrajectory, queue_size=5)

    
    global joint_trajectory_precision
    joint_trajectory_precision = rospy.Publisher(
        '/figment/trajectory/precision', std_msgs.msg.String, queue_size=5)
        
    global figment_action_pub
    figment_action_pub = rospy.Publisher(
        '/figment/action', std_msgs.msg.String, queue_size=1)

    joint_trajectory_precision.publish("Low")
    figment_action_pub.publish("start")

    rospy.set_param('go_to_initial_position', 'Low')
    rospy.set_param('go_to_tray_position', 'High')
    #rospy.set_param('go_to_tray_position', 'Low')
    rospy.set_param('go_to_belt_start', 'Low')
    rospy.set_param('go_to_belt', 'Low')
    rospy.set_param('go_to_part_bin_front', 'Low')
    rospy.set_param('go_to_bin_front', 'Low')
    rospy.set_param('go_to_position_a_bit_above_part', 'High')
    rospy.set_param('go_down_until_get_piece', 'High')
    rospy.set_param('turnWrist', 'High')
    rospy.set_param('MoveSideWays', 'Low')
    rospy.set_param('moveToolTip', 'High')
    rospy.set_param('moveToolTipZY', 'High')

    rospy.set_param('discard_part_from_belt', 'Low')
    rospy.set_param('go_discard_from_tray1', 'Low')
    rospy.set_param('go_to_discard_open_bin', 'Low')
    rospy.set_param('turn_pulley_yellow_bar', 'High')
    
    global change_latency_cmd
    change_latency_cmd='sudo -S tc qdisc change dev enp5s0 root netem  delay {delay}ms'
    
    #~ global sudo_pw
    #~ sudo_pw = os.environ.get('SUDO_PASSWORD')
    #~ if sudo_pw is None:
        #~ print("Export your sudo password to SUDO_PASSWORD envvar")
        #~ sys.exit(1)
    

    #for 'move_towards_piece_on_belt' check its code 
    
def set_latency(delay):
    cmd=change_latency_cmd.format(delay=delay).split()
    p=subprocess.Popen(cmd, stderr=subprocess.STDOUT, stdin=subprocess.PIPE, shell=False)
    p.communicate(input=sudo_pw+"\n")
    rospy.logwarn("[set_latency]: Set latency to "+str(delay))    
    

def set_trajectory_precision(action):  
    value = rospy.get_param(action, "NotDefined")
    figment_action_pub.publish(action)
    if value is "NotDefined":
        rospy.loginfo("[set_trajectory_precision]: No value set for: \" " + action + " \". Using High.")    
        value =  "High"
    rospy.loginfo("[set_trajectory_precision]: Setting precision: " + value)
    joint_trajectory_precision.publish(value)
    #set_latency(20 if value=='Low' else 1)

def set_trajectory_precision_value(value): 
    rospy.loginfo("[set_trajectory_precision]: Setting precision: " + value)
    joint_trajectory_precision.publish(value)
    #set_latency(20 if value=='Low' else 1)





def go_to_initial_position():
    """
    Move the arm to the initial position defined statically in constants
    """
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    
    set_arm_joint_values(list_of_joint_values=STATIC_POSITIONS[
                         "initial_position"], time_to_execute_action=0.5)
    

def go_to_tray_position(tray_id, time=2):
    """
    Move the arm to static position in front of the tray
    tray_id - key for the wanted tray to be used in the STATIC_POSITIONS MAP.
    """
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    set_arm_joint_values(STATIC_POSITIONS[tray_id], time)
    

    return STATIC_POSITIONS[tray_id]

def go_to_belt_start():
    """
    Move to the static positon in front of the belt.

    """    
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    rest_position_start = STATIC_POSITIONS["beltStart"]
    #linear_arm_actuator_joint = part.y

    set_arm_joint_values(rest_position_start, 1)
    return rest_position_start

def go_to_belt():
    """
    Move to the static positon in front of the belt.

    """   
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    rest_position = STATIC_POSITIONS["belt"]
    #linear_arm_actuator_joint = part.y

    set_arm_joint_values(rest_position, 1)
    return rest_position

def go_to_part_bin_front(part_world_position):
    """
    Move to the front of the wanted part.
    part_world_position : world coordinates of the part position.

    """
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    rest_position = STATIC_POSITIONS["rest_position"]
    #linear_arm_actuator_joint = part.y

    rest_position[1] = part_world_position[1] + WRIST_LENGTH

    set_arm_joint_values(rest_position, 1)
    return rest_position

def go_to_bin_front(bin_id):
    """
    Move to the front of the wanted bin_id.
    bin_id : key for the wanted bin to be used in the STATIC_POSITIONS MAP.
    """
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    set_arm_joint_values(STATIC_POSITIONS[bin_id], 1)


def go_to_position_a_bit_above_part(world_position, world_orientation, part_type, time_to_execute_action, solver_type, a_bit_above_value=0.015, ignore_height=False, adjust=False):
    """
        Move the arm to a position a little bit above the part.

        world_position - world coordinates of the part position
        world_orientation - world orientation (rotation) of the part
        part_type - part type (piston_rod, gear, ...)
        time_to_execute_action - time to execute the moviment
        solver_type - value to select which solver to use (tray, bin, agv1, agv2)
        a_bit_above_value - distance value that the arm will be above the part

    """
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    world_position_above_part = [world_position[0], world_position[1], world_position[2] + a_bit_above_value]

    rospy.loginfo("[go_to_position_a_bit_above_part] world_position_above_part = " + str(world_position_above_part))

    angles = []
    if solver_type == SolverType.BIN:
        angles = solverBin(
            world_position_above_part, world_orientation, part_type, ignore_height)
    elif solver_type == SolverType.AGV1:
        world_position_above_part[2] += 0.05
        angles = depositOnTray1(
            world_position_above_part, world_orientation, part_type, adjust=adjust)
    elif solver_type == SolverType.AGV2:
        world_position_above_part[2] += 0.05
        angles = depositOnTray2(
            world_position_above_part, world_orientation, part_type)
    elif solver_type == SolverType.BELT:
        angles = solverBelt(
            world_position_above_part, world_orientation, part_type)
    
    set_arm_joint_values(angles, time_to_execute_action)
    rospy.sleep(0.1)
    return angles

def turn_pulley_yellow_bar(self, part_world_orientation):
    set_trajectory_precision(inspect.currentframe().f_code.co_name)

    rospy.loginfo("[turn_pulley_yellow_bar] Turning pulley at yellow bar")
    

    rospy.loginfo("[turn_pulley_yellow_bar] Move to bin8")
    angles = STATIC_POSITIONS["bin8"]
    arm_actions.set_arm_joint_values(angles, 3)

    rospy.loginfo("[turn_pulley_yellow_bar] Move Up 1")
    angles = [2.62, 1.15, -1.80, 3.14, 3.34, -1.57, 0]
    set_arm_joint_values(angles, 1)

    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=True, force_grp_sts=True)

    rospy.loginfo("[turn_pulley_yellow_bar] Move Up 2")
    angles = [2.62, 1.15, -2.18, 3.14, 3.34, 1.57, 0]
    set_arm_joint_values(angles, 1)
    
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                                force_check_piece=True, force_grp_sts=True)
    

    angles = [1.61, 1.15, -2.18, 3.14, 5.03, 1.57, 0]
    set_arm_joint_values(angles, 1)
    
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=True, force_grp_sts=True)

    angles = [1.44, 1.15, -1.79, 3.14, 5.03, 1.57, 0]
    set_arm_joint_values(angles, 1)
    rospy.loginfo("[turn_pulley_yellow_bar] Place 1")
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=True, force_grp_sts=True)

    
    gripper_actions.send_gripping_cmd(toGrip=False)


    angles = [1.50, 1.15, -1.81, 3.14, 5.03, 1.57, 0]
    set_arm_joint_values(angles, 1)
    rospy.loginfo("[turn_pulley_yellow_bar] Out 1")
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=False, force_grp_sts=True)

    angles = [1.75, 1.15, -2.19, 3.14, 5.03, 1.57, 0]
    set_arm_joint_values(angles, 1)
    rospy.loginfo("[turn_pulley_yellow_bar] Out 2")
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=False, force_grp_sts=True)

    angles = [1.00, 1.15, -1.90, 3.14, 5.03, -1.57, 0]
    set_arm_joint_values(angles, 1)
    rospy.loginfo("[turn_pulley_yellow_bar] Out 3")
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=False, force_grp_sts=True)

    angles = [1.00, 1.15, -1.90, 3.14, 5.03, -1.57, 0]
    set_arm_joint_values(angles, 1)
    rospy.loginfo("[turn_pulley_yellow_bar] Adjust 1")
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=False, force_grp_sts=True)

    angles = [0.50, 1.15, -1.50, 3.14, 5.60, -1.57, 0]
    set_arm_joint_values(angles, 1)
    rospy.loginfo("[turn_pulley_yellow_bar] Adjust 2")
    check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=False, force_grp_sts=True)

    #angles = [0.10, 1.18, -1.19, 3.14, 5.80, -1.57, 0]
    #self.moveTo(angles, 4)
    #teste = ActionSleep(3, "Take")
    # teste.execute_action()

    gripper_actions.send_gripping_cmd(toGrip=True)
    angles = [-0.13, 1.18, -1.08, 3.14, 5.92, -1.57, 0]
    set_arm_joint_values(angles, 4)
    rospy.loginfo("[turn_pulley_yellow_bar] Take Test")

    gripper_actions.wait_for_gripper(toGrip=True, max_wait=5, inc_sleep=0.005)
    
    angles = [0.11, 1.18, -1.24, 3.14, 5.68, -1.57, 1.57 - part_world_orientation[2]]
    set_arm_joint_values(angles, 1)
    rospy.loginfo("[turn_pulley_yellow_bar] Out with Pulley")

    success = check_arm_joint_values_published(list_of_joint_values=angles, 
                                    force_check_piece=True, force_grp_sts=True)
    if success:

        angles = [1.49, 1.18, -2.25, 3.14, 5.18, -1.57, 1.57 - part_world_orientation[2]]
        set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Adjust 1")
        
        success = check_arm_joint_values_published(list_of_joint_values=angles, 
                            force_check_piece=True, force_grp_sts=True)


        angles = [2.12, 1.18, -2.12, 3.14, 4.30, -1.57, 1.57 - part_world_orientation[2]]
        set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Adjust 2")

        success = check_arm_joint_values_published(list_of_joint_values=angles, 
                            force_check_piece=True, force_grp_sts=True)

        return success

    return False

def go_to_discard_open_bin(tray_id):

    set_trajectory_precision(inspect.currentframe().f_code.co_name)

    if(tray_id == 1):
        angles_discard = STATIC_POSITIONS["disBelAgv1"]
        angles_discard_open = STATIC_POSITIONS["disBelAgv1Open"]
        angles_discard_back = STATIC_POSITIONS["disBelAgv1Back"]
        rospy.loginfo("[Actions] - Go to discard pos")
        arm_actions.go_discard_from_tray1(list_of_joint_values=angles_discard,
                                                            time_to_execute_action=0.5)
    elif(tray_id == 2): 
        angles_discard = STATIC_POSITIONS["disBelAgv2"]
        angles_discard_open = STATIC_POSITIONS["disBelAgv2Open"]
        angles_discard_back = STATIC_POSITIONS["disBelAgv2Back"]






    rospy.loginfo("[Actions] - Go to discard open pos")
    set_arm_joint_values(list_of_joint_values=angles_discard_open,
        time_to_execute_action=0.5)

    check_arm_joint_values_published(list_of_joint_values=angles_discard_open)


    rospy.loginfo("[Actions] - discard pos") 
    success = gripper_actions.send_gripping_cmd_and_wait(False)
    rospy.loginfo("[Actions]  send_gripping_cmd_and_wait success: {}".format(success)) 

    
    set_arm_joint_values(list_of_joint_values=angles_discard_back,
        time_to_execute_action=0.5)

    check_arm_joint_values_published(list_of_joint_values=angles_discard_back)

    return success

def move_towards_piece_on_belt(part_world_position, part_world_orientation,
                                    part_world_tf_time, part_type):
        
    #move_belt_part1
    set_trajectory_precision_value("Low")
    figment_action_pub.publish('move_towards_piece_on_belt_part1')

    camera_pos = global_vars.tf_manager.get_transform('world', "logical_camera_belt_1_frame").translation

    pos_robot = list(global_vars.current_joint_state.position)
    print(part_world_orientation)
    list_joint_values = solverBelt(part_world_position, part_world_orientation, part_type)
    initial_position = deepcopy(list_joint_values)

    now = rospy.get_time()
    timer = rospy.get_time()
    time_diff = (now-part_world_tf_time.to_sec())
    print "a_time: " + str(now) + " trans_time: " + str(part_world_tf_time.to_sec()) + " diff = " + str(time_diff)
    
    incr = time_diff * 0.2  # if time_diff > 2 else 0.2

    # t = rospy.get_time()
    # timer = rospy.get_time()
    # time_diff = (t-part_world_tf_time)
    # print "a_time: " + str(t) + " trans_time: " + str(part_world_tf_time) + " diff = " + str(time_diff)
    # incr = time_diff * 0.2  # if time_diff > 2 else 0.2
    # wrist_comp = WRIST_1_2 * 0.33 if time_diff > 2 else WRIST_1_2/2
    
    pos_robot[0] = list_joint_values[0] 
    pos_robot[1] = list_joint_values[1] - incr
    pos_robot[3] = list_joint_values[3] 
    pos_robot[4] = list_joint_values[4] 
    pos_robot[5] = list_joint_values[5]
    pos_robot[6] = list_joint_values[6]

    if (pos_robot[1] <= -2.1):
        return False


    set_arm_joint_values(pos_robot, time_diff)
    #move_belt_part1
    set_trajectory_precision_value("High")
    figment_action_pub.publish('move_towards_piece_on_belt_part2')


    back_position = pos_robot[2]
    while not global_vars.gripper_state.attached:
        gripper_actions.send_gripping_cmd(toGrip=True)
        pos_robot[1] -= 0.2 

        if pos_robot[2] < list_joint_values[2]:
            pos_robot[2] += 0.08 #TODO increase

        if pos_robot[2] > list_joint_values[2]:
            pos_robot[2] = list_joint_values[2]

        set_arm_joint_values(pos_robot, 1)

        if rospy.get_time()-timer >= 20 or pos_robot[1] <= -2.1:
            return False

        rospy.sleep(1)

    set_trajectory_precision_value("Low")
    figment_action_pub.publish('move_towards_piece_on_belt_part3')
    # raising arm again
    pos_robot[0] = STATIC_POSITIONS["rest_position"][0]
    pos_robot[2] = back_position
    set_arm_joint_values(pos_robot, 0.5)
    rospy.sleep(0.5)#TODO can we change to check_angles?

    return True

def go_discard_from_tray1(list_of_joint_values, time_to_execute_action):
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    rospy.loginfo("[Actions] - Go to discard pos")
    set_arm_joint_values(list_of_joint_values=list_of_joint_values,
                            time_to_execute_action=time_to_execute_action)
    check_arm_joint_values_published(list_of_joint_values=list_of_joint_values)


def discard_part_from_belt(list_of_joint_values, time_to_execute_action):

    set_trajectory_precision(inspect.currentframe().f_code.co_name)

    rospy.loginfo("[Actions] - Go to discard pos") 
    set_arm_joint_values(list_of_joint_values=list_of_joint_values,
                        time_to_execute_action=time_to_execute_action)
    

    check_arm_joint_values_published(list_of_joint_values=angles_discard)

    rospy.loginfo("[Arm Actions] - discard from bel pos") 
    success = gripper_actions.send_gripping_cmd_and_wait(False) 
    return success

def go_down_until_get_piece(world_position, world_orientation, part_type, 
    time=3, distance=0.01, solver_type=SolverType.BIN, ignore_height=False, adjust=False):

    set_trajectory_precision(inspect.currentframe().f_code.co_name)    
    rospy.loginfo("[Arm Actions] go_down_until_get_piece")
    tfPos = world_position
    tfOri = world_orientation
    position = [tfPos[0], tfPos[1], tfPos[2] - distance]
    position2 = [tfPos[0], tfPos[1], tfPos[2] + 0.02]

    rospy.loginfo(
        "[go_down_until_get_piece] goDownUntilGetPiece pos = " + str(position))
    try:
        if solver_type == SolverType.BIN:
            angles = solverBin(
                position, tfOri, part_type, ignore_height)
            angles[4]+=0.05
            angles[5]+=0.03
            angles2 = solverBin(
                position2, tfOri, part_type, ignore_height)
            angles2[4]+=0.05

        
        elif solver_type == SolverType.AGV1:  
            rospy.loginfo(
            "[go_down_until_get_piece] SolverType.AGV1" )
            angles = depositOnTray1(
                position, tfOri, part_type, ignore_height=ignore_height, adjust=adjust)
            angles2= depositOnTray1(
                position2, tfOri, part_type, ignore_height=ignore_height, adjust=adjust) 
        elif solver_type == SolverType.AGV2:
            angles = depositOnTray2(
                position, tfOri, part_type, ignore_height=ignore_height)
            angles2 = depositOnTray2(
                position2, tfOri, part_type, ignore_height=ignore_height)        
    except AssertionError:
        rospy.logerr(
        "[go_down_until_get_piece] Maths Failure, couldn't find triangle")
        return False



    success = gripper_actions.send_gripping_cmd(toGrip=True)
    set_arm_joint_values(angles, time)
    if not success:
        rospy.logerr(
        "[go_down_until_get_piece] send_gripping_cmd Failure")
        return False

    success = gripper_actions.wait_for_gripper(toGrip=True, max_wait=time+1, inc_sleep=0.005)
    if not success:
        rospy.logerr(
        "[go_down_until_get_piece] wait_for_gripper Failure")
        return False

    set_arm_joint_values(angles2, 0.1)
    rospy.sleep(0.1)
    return True

    


def set_arm_joint_values(list_of_joint_values, time_to_execute_action):
    """
        Publish the joint values as a JointTrajectory Object
    """
    
    msg = utils.createJointTrajectory(
        list_of_joint_values, (1/global_vars.speed_modifier) * time_to_execute_action)
    rospy.loginfo("[set_arm_joint_values]: Setting joint values " +
                  str(list_of_joint_values) + " in the following order " + str(ARM_JOINT_NAMES))
    joint_trajectory_publisher.publish(msg)


def check_arm_joint_values_published(list_of_joint_values=None, static_position_key=None,
                                     accError=[0.009, 0.009, 0.009, 0.009,
                                               0.015, 0.015, 0.009, 0.009, 0.009],
                                     max_sleep=8, force_check_piece=False, force_grp_sts=True):
    """
    Check if the actual state values of the joints is equal to the list joint states of the expected position.

    list_of_joint_values - list of expected joint values
    static_position_key - key to get a static position from the STATIC_POSITION map
    accError - Acceptable error for each joint
    """
    
    final_joint_values = list_of_joint_values if list_of_joint_values is not None else STATIC_POSITIONS[static_position_key]

    rospy.loginfo("[check_arm_joint_values_published]: final_joint_values:" + str(final_joint_values)+ str(len(final_joint_values)))
    inc_sleep = 0.05
    slept = 0
    result = False
    grpOK = True
    
    lastError = None
    
    # multiplies the acceptable error for each joint
    while not result and slept < max_sleep and grpOK:
        position = global_vars.current_joint_state.position
        #rospy.loginfo("Error: " + str(map(operator.sub, position[:7], final_joint_values)))
        result, listRest = utils.comparePosition(
             #position, final_joint_values, [e * goalToleranceRatio for e in accError]) 
             position, final_joint_values, accError) 
        if(lastError!=listRest):
            rospy.loginfo("Error: " + str(listRest))	
        lastError=listRest
        if(not result):
            rospy.sleep(inc_sleep)
            slept += inc_sleep
            if(force_check_piece):
                grpOK = (global_vars.gripper_state.attached == force_grp_sts)    

    if(not result):
        rospy.logerr("[check_arm_joint_values_published] - Goal not reached")
        rospy.logerr(
            "[check_arm_joint_values_published] - List Joint: " + str(listRest))
        rospy.logerr("[check_arm_joint_values_published] - Expected position " +
                     str(final_joint_values) + " | Actual position " + str(position))
        return False
    else:
        if (not grpOK):
            rospy.logerr("[check_arm_joint_values_published] - Gripper Failed")
            return False
        else:    
            rospy.loginfo("[check_arm_joint_values_published] - Goal Reached")
            return True


def turnWrist(turn_wrist):
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    position = global_vars.current_joint_state.position
    angles = []
    angles.extend(position)
    angles[5] = turn_wrist

    set_arm_joint_values(angles, 0.1)

    check_arm_joint_values_published(
        list_of_joint_values=angles)


def MoveSideWays(move_side):
    set_trajectory_precision(inspect.currentframe().f_code.co_name)        
    position = global_vars.current_joint_state.position
    angles = []
    angles.extend(position)
    angles[1] = angles[1] + move_side 
    
    set_arm_joint_values(angles, 1.2)

    check_arm_joint_values_published(
        list_of_joint_values=angles)
        

def moveToolTip(incrementZ=0.3, incrementX=0.1, timeToGoal=0.2, 
                accError=[0.009, 0.009, 0.009, 0.009,
                                               0.015, 0.015, 0.009, 0.009, 0.009]):
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
        
    posUpperArm, angleUpperArm = utils.getUpperArmPose()
    posVacum, angleVacum = utils.getVacuumGripperPos()
    posFore, angleFore = utils.getForeArmPos()

    workingPos = deepcopy(posVacum)
    workingPos[2] += incrementZ
    workingPos[0] += incrementX


    shoulderBTriangle = utils.computeXZDistance(posUpperArm, workingPos)

    xDistance = abs(utils.computeXDistance(posUpperArm, workingPos))
    zDistance = abs(utils.computeZDistance(posUpperArm, workingPos))


    wristBTriangle = utils.computeXZDistance(workingPos, posVacum)

    workingIsAbove = utils.computeZDistance(posUpperArm, workingPos) < 0

    a1, b1, c1, A1, B1, C1 = solve(
        a=shoulderBTriangle, b=FORE_ARM, c=UP_ARM)

    a2, b2, c2, A2, B2, C2 = solve(
        a=xDistance, b=zDistance, c=shoulderBTriangle)

    elbow_joint = math.pi - A1

    shoulder_lift_joint = math.pi / 2 - A2 -B1
    wrist_1_joint =  math.pi - (C1 + B2) + math.pi/2
 
    angles = []
    angles.extend(global_vars.current_joint_state.position)
    angles[0] = elbow_joint
    angles[2] = shoulder_lift_joint
    angles[4] = wrist_1_joint

    # rospy.sleep(1)

    set_arm_joint_values(angles, timeToGoal)

    check_arm_joint_values_published(
        list_of_joint_values=angles, accError=accError)


            
def moveToolTipZY(incrementZ=0.3, incrementY=0.1, timeToGoal=0.2): 
    set_trajectory_precision(inspect.currentframe().f_code.co_name)
    posUpperArm, angleUpperArm = utils.getUpperArmPose()
    posVacum, angleVacum = utils.getVacuumGripperPos()
    posFore, angleFore = utils.getForeArmPos()

    workingPos = deepcopy(posVacum)
    workingPos[2] += incrementZ
    workingPos[1] += incrementY


    shoulderBTriangle = utils.computeYZDistance(posUpperArm, workingPos)

    yDistance = abs(utils.computeYDistance(posUpperArm, workingPos))
    zDistance = abs(utils.computeZDistance(posUpperArm, workingPos))


    wristBTriangle = utils.computeYZDistance(workingPos, posVacum)

    workingIsAbove = utils.computeZDistance(posUpperArm, workingPos) < 0

    a1, b1, c1, A1, B1, C1 = solve(
        a=shoulderBTriangle, b=FORE_ARM, c=UP_ARM)

    a2, b2, c2, A2, B2, C2 = solve(
        a=yDistance, b=zDistance, c=shoulderBTriangle)

    elbow_joint = math.pi - A1

    shoulder_lift_joint = math.pi / 2 - A2 -B1
    wrist_1_joint =  math.pi - (C1 + B2) + math.pi/2
 
    angles = []
    angles.extend(global_vars.current_joint_state.position)
    angles[0] = elbow_joint
    angles[2] = shoulder_lift_joint
    angles[4] = wrist_1_joint

    # rospy.sleep(1)

    # msg = utils.createJointTrajectory(angles, time=1)
    # joint_trajectory_publisher.publish(msg)
    set_arm_joint_values(angles, timeToGoal)

    check_arm_joint_values_published(
        list_of_joint_values=angles)



