import rospy
import utils
import global_vars
from osrf_gear.srv import VacuumGripperControl

from constants import *


def send_gripping_cmd(toGrip):
    rospy.wait_for_service("/ariac/gripper/control")
    try:
        gripping = rospy.ServiceProxy(
            "/ariac/gripper/control", VacuumGripperControl)
        success = gripping(toGrip)
        global_vars.partPicked = toGrip
        return success

    except rospy.ServiceException as exc:
        rospy.logerr("[GripperActions] Failed to use gripper: %s" % exc)
        return False



def wait_for_gripper(toGrip, max_wait, inc_sleep=0.1):
    rospy.wait_for_service("/ariac/gripper/control")
    try:
        count = 1
        slept = 0
        rospy.loginfo("[GripperActions]-wait_for_gripper Cur/Des " + 
            str(global_vars.gripper_state.attached) + str(toGrip))
        success = False
        while(global_vars.gripper_state.attached is not toGrip):
            if(slept >= max_wait):
                break
            rospy.sleep(inc_sleep)
            rospy.loginfo_throttle(1, "[GripperActions] current: " + str(global_vars.gripper_state.attached)) 
            slept += inc_sleep

        success = global_vars.gripper_state.attached is toGrip
        rospy.loginfo(
                "[wait_for_gripper] success: {}".format(success))
        return success
    except rospy.ServiceException as exc:
        rospy.logerr("[GripperActions] Wait for gripper failed: %s" % exc)
        return False

def send_gripping_cmd_and_wait(toGrip, max_wait=5, inc_sleep=0.01):
    success = send_gripping_cmd(toGrip)
    rospy.loginfo(
        "[send_gripping_cmd_and_wait] send_gripping_cmd success: {}".format(success))
    print(success)
    if not success:
        rospy.logerr(
        "[send_gripping_cmd_and_wait] send_gripping_cmd Failure")
        return False

    success = wait_for_gripper(toGrip=toGrip, max_wait=max_wait, inc_sleep=inc_sleep)
    rospy.loginfo(
        "[send_gripping_cmd_and_wait] wait_for_gripper success: {}".format(success))   
    if not success:
        rospy.logerr(
        "[send_gripping_cmd_and_wait] wait_for_gripper Failure")
        return False
    return success