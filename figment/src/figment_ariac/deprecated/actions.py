import sys
import rospy
import time
import tf2_ros
import math
import tf as transf
from copy import deepcopy

from osrf_gear.msg import VacuumGripperState, LogicalCameraImage, ConveyorBeltState
from osrf_gear.srv import AGVControl, VacuumGripperControl, GetMaterialLocations, ConveyorBeltControl

import utils
from ik_solution import solverBin, solverBelt, depositOnTray2, depositOnTray1, moveUp, computeToolTipGoal

from constants import *
import global_vars
from trianglesolver import solve, degree
import transform


class Action:

    def __init__(self):
        pass

    def execute_action(self):
        pass


class ActionCameraTF(Action):

    def __init__(self, kit_object, destination):
        global BIN_CAMERA
        self.material = kit_object.type
        self.kit_object = kit_object
        self.destination = destination
        self.calPos = None
        self.calOri = None
        self.calTime = None
        self.camera = BIN_CAMERA[self.destination]
        self.part_name = self.camera + "_" + self.material + "_"

    def execute_action(self):
        
        camera_id, part_id = global_vars.tf_manager.find_part_name(
            self.part_name, dad=self.camera+"_frame")

        # we did find the frames
        if len(camera_id) > 0 or len(part_id) > 0:
            rospy.logerr("camera_id %s" % camera_id)
            rospy.logerr("part_id %s" % part_id)
            camera_transform = global_vars.tf_manager.get_transform(
                'world', camera_id)
            part_transform = global_vars.tf_manager.get_transform(
                camera_id, part_id)

            if camera_transform is not None and part_transform is not None:
                self.calPos, self.calOri = transform.transform_to_world(
                    part_transform, camera_transform)
                # self.calOri = transf.transformations.quaternion_from_euler(self.calOri[0], self.calOri[1], self.calOri[2])
                self.calTime = global_vars.tf_manager.transforms_dynamic[camera_id][part_id]["secs"]
                global_vars.tf_manager.remove_transform(camera_id, part_id)

                return self.calPos, self.calOri

            rospy.logerr("part_transform %s" % part_transform)
            rospy.logerr("camera_transform %s" % camera_transform)

        rospy.logerr("Could not find kit_object:" + str(self.kit_object))


class ActionCameraTFTurn(Action):

    def __init__(self, kit_object, part_name):
        self.material = kit_object.type
        self.kit_object = kit_object
        self.part_name = part_name
        self.calPos = None
        self.calOri = None
        self.calTime = None

    def execute_action(self):

        #rospy.loginfo("[ActionCameraTF] - camera = " + str(camera))
        #rospy.loginfo("[ActionCameraTF] - part_name = " + str(self.part_name))
        #rospy.loginfo("[ActionCameraTF] - tf_transforms = " + str(tf_transforms.items()))

        camera_id, part_id = global_vars.tf_manager.find_part_name(
            self.part_name)

        # we did find the frames
        if len(camera_id) > 0 or len(part_id) > 0:
            camera_transform = global_vars.tf_manager.get_transform(
                'world', camera_id)
            part_transform = global_vars.tf_manager.get_transform(
                camera_id, part_id)
            self.calPos, self.calOri = transform.transform_to_world(
                part_transform, camera_transform)
            self.calTime = global_vars.tf_manager.transforms_dynamic[camera_id][part_id]["secs"]
            
            return self.calPos, self.calOri
        else:
            rospy.logerr("Could not find kit_object:" + str(self.kit_object))


class ActionMaterialLocation(Action):
    service_name = "/ariac/material_locations"

    def __init__(self, _type, kit_object, kit_type, text="DefaulMaterial"):
        self._type = _type
        self.kit_object = kit_object
        self.kit_type = kit_type
        self.text = text

    def execute_action(self):
        rospy.wait_for_service(ActionMaterialLocation.service_name)
        try:
            material_location = rospy.ServiceProxy(
                ActionMaterialLocation.service_name, GetMaterialLocations)
            location = material_location(self._type)
            return location.storage_units
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to get the material location: %s" % exc)
            return []

    def __str__(self):
        return "ActionMaterial: " + self.text

    def __repr__(self):
        return "<ActionMaterial _type:%s kit_type:%s text:%s>" % (self._type, self.kit_type, self.text)


class ActionPose(Action):

    def __init__(self, pose, joint_trajectory_publisher, time):
        self.pose = pose
        self.angles = pose
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time

    def execute_action(self):
        msg = utils.createJointTrajectory(angles=self.pose, time=self.time)
        # rospy.loginfo("Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)
        # rospy.sleep(self.time)
        rospy.sleep(0.2)

class ActionGoDownUntilGetPiece(Action):


    def __init__(self, positionAct, joint_trajectory_publisher, object_type, time=3, ignoreHeight=False, distance=0.03):
        #rospy.loginfo("[ActionMove] self = " + str(self))
        self.positionAct = positionAct
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time
        self.object_type = object_type
        self.ignoreHeight = ignoreHeight
        self.distance = distance

    # def _setPosition(self, position):
    #     rospy.loginfo("[ActionMove] ####### ALGUEM ACESSOU #########")
    #     self.position = position

    def waitForGripper(self, cmdSent, toGrip, inc_sleep=0.1):
        if(cmdSent):  # If the command could be executed
            rospy.logerr(
                "[ActionGoDownUntilGetPiece] - Waiting while gripper state is " + str(toGrip))
            checkGripperState = ActionGrippingCheck(
                toGrip=toGrip, max_wait=5, inc_sleep=inc_sleep)
            sucess = checkGripperState.execute_action()
        else:
            rospy.logerr("[ActionGoDownUntilGetPiece] - Severe ERROR.\n" +
                         "We were not able to send the command to the gripper control service")

        if(not sucess):  # Sucess mean the command was executed
            rospy.logerr("[ActionGoDownUntilGetPiece] - Severe ERROR.\n" +
                         "We should have attached the gripper already")
            # TODO - Do something. Like adding action back
    def execute_action(self):
        
        rospy.loginfo("[ActionGoDownUntilGetPiece] goDownUntilGetPiece")
        tfPos = self.positionAct.calPos
        tfOri = self.positionAct.calOri
        position = [tfPos[0], tfPos[1], tfPos[2] - self.distance]

        # rospy.sleep(self.time + 2) #DECREASE
        rospy.loginfo(
            "[ActionGoDownUntilGetPiece] goDownUntilGetPiece pos = " + str(position))
        angles = solverBin(position, tfOri, self.object_type,
                           ignoreHeight=self.ignoreHeight)

        position2 = [tfPos[0], tfPos[1], tfPos[2] + 0.02]
        angles2 = solverBin(position2, tfOri, self.object_type,
                            ignoreHeight=self.ignoreHeight)

        msg = utils.createJointTrajectory(angles, self.time)
        msg2 = utils.createJointTrajectory(angles2, 1)
        self.joint_trajectory_publisher.publish(msg)
        sendCmdToGripper = ActionCmdGripping(toGrip=True)
        cmdSent = sendCmdToGripper.execute_action()
        # Wait while the gripper attaches
        # returns as soon as the gripper attaches
        self.waitForGripper(cmdSent=cmdSent, toGrip=True, inc_sleep=0.01)
        self.joint_trajectory_publisher.publish(msg2)
        rospy.sleep(0.1)



class ActionMoveABitAbovePiece(Action):
    BIN, AGV1, AGV2, BELT = range(4)

    def __init__(self, positionAct, joint_trajectory_publisher, time, object_type, solverType, ignoreHeight=False, distance=0.03):
        #rospy.loginfo("[ActionMove] self = " + str(self))
        self.positionAct = positionAct
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time
        self.object_type = object_type
        self.solverType = solverType
        self.ignoreHeight = ignoreHeight
        self.distance = distance

    # def _setPosition(self, position):
    #     rospy.loginfo("[ActionMove] ####### ALGUEM ACESSOU #########")
    #     self.position = position

    def execute_action(self):

        self.calPos = deepcopy(self.positionAct.calPos)
        self.calPos[2] += self.distance
        self.calOri = self.positionAct.calOri

        self.position = [self.positionAct.calPos[0],
                         self.positionAct.calPos[1], self.positionAct.calPos[2] + self.distance]

        # rospy.sleep(self.time + 2) #DECREASE
        rospy.loginfo("[ActionMoveABitAbovePiece] pos = " +
                      str(self.position) + "  self = " + str(self))
        if self.solverType == ActionMove.BIN:
            self.angles = solverBin(
                self.position, self.calOri, self.object_type, self.ignoreHeight)
        elif self.solverType == ActionMove.AGV1:
            self.angles = depositOnTray1(
                self.position, self.calOri, self.object_type)
        elif self.solverType == ActionMove.AGV2:
            self.angles = depositOnTray2(
                self.position, self.calOri, self.object_type)
        elif self.solverType == ActionMove.BELT:
            self.angles = solverBelt(
                self.position, self.calOri, self.object_type)
        msg = utils.createJointTrajectory(self.angles, self.time)
        #rospy.loginfo("[ActionMove] Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)
        # rospy.sleep(self.time + 2)


class ActionMovePiece(Action):
    BIN, AGV1, AGV2, BELT = range(4)

    def __init__(self, positionAct, joint_trajectory_publisher, time, object_type, solverType, ignoreHeight=False, gprt=None):
        #rospy.loginfo("[ActionMove] self = " + str(self))
        self.positionAct = positionAct
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time
        self.object_type = object_type
        self.solverType = solverType
        self.ignoreHeight = ignoreHeight
        self.gprt = gprt

    # def _setPosition(self, position):
    #     rospy.loginfo("[ActionMove] ####### ALGUEM ACESSOU #########")
    #     self.position = position

    def execute_action(self):
        self.position = self.positionAct.calPos
        rotV = self.positionAct.calOri
        tfTimer = self.positionAct.calTime
        # euler_from_quternion() returns a tuple
        # rotV = transf.transformations.euler_from_quaternion(rotV)

        # rospy.sleep(self.time + 2) #DECREASE
        rospy.loginfo("[execute_action] pos = " +
                      str(self.position) + "  self = " + str(self))
        if self.solverType == ActionMove.BIN:
            self.angles = solverBin(
                self.position, rotV, self.object_type, self.ignoreHeight)
            msg = utils.createJointTrajectory(self.angles, self.time)
            #rospy.loginfo("[ActionMove] Sending command:\n" + str(msg))
            self.joint_trajectory_publisher.publish(msg)
        elif self.solverType == ActionMove.AGV1:
            self.angles = depositOnTray1(self.position, rotV, self.object_type)
            msg = utils.createJointTrajectory(self.angles, self.time)
            #rospy.loginfo("[ActionMove] Sending command:\n" + str(msg))
            self.joint_trajectory_publisher.publish(msg)
        elif self.solverType == ActionMove.AGV2:
            self.angles = depositOnTray2(self.position, rotV, self.object_type)
            msg = utils.createJointTrajectory(self.angles, self.time)
            #rospy.loginfo("[ActionMove] Sending command:\n" + str(msg))
            self.joint_trajectory_publisher.publish(msg)
        elif self.solverType == ActionMove.BELT:
            self.angles = solverBelt(self.position, rotV, self.object_type)
            #S_f = -V * Delta_t + S_i
            final = (-0.2*(rospy.get_time()+1-tfTimer)) + (self.angles[1])
            position = list(self.gprt.current_joint_state.position)
            position[1] = final
            self.angles[1] = final
            print final, rospy.get_time()-tfTimer, tfTimer
            msg = utils.createJointTrajectory(position, 1)
            self.joint_trajectory_publisher.publish(msg)

            rospy.sleep(rospy.get_time()-tfTimer)
            gripping = ActionCmdGripping(True)
            gripping.execute_action()
            while not global_vars.gripper_state.attached:
                rospy.sleep(0.1)
                self.angles[1] -= 0.02
                msg = utils.createJointTrajectory(self.angles, 1)
                self.joint_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)
        # rospy.sleep(self.time + 2)


class ActionMove(Action):
    BIN, AGV1, AGV2, BELT = range(4)

    def __init__(self, position, rotation, joint_trajectory_publisher, time, object_type, solverType):
        #rospy.loginfo("[ActionMove] self = " + str(self))
        self.position = position
        self.rotation = rotation
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time
        self.object_type = object_type
        self.solverType = solverType
        self.angles = None

    # def _setPosition(self, position):
    #     rospy.loginfo("[ActionMove] ####### ALGUEM ACESSOU #########")
    #     self.position = position

    def execute_action(self):
        # rospy.sleep(self.time + 2) #DECREASE
        #rospy.loginfo("[execute_action] pos = " + str(self.position) + "  self = " + str(self))
        if len(self.rotation) == 4:
            self.rotation = transf.transformations.euler_from_quaternion(
                self.rotation)
        if self.solverType == ActionMove.BIN:
            self.angles = solverBin(
                self.position, self.rotation, self.object_type)
        elif self.solverType == ActionMove.AGV1:
            self.angles = depositOnTray1(
                self.position, self.rotation, self.object_type)
        elif self.solverType == ActionMove.AGV2:
            self.angles = depositOnTray2(
                self.position, self.rotation, self.object_type)
        elif self.solverType == ActionMove.BELT:
            self.angles = solverBelt(
                self.position, self.rotation, self.object_type)
        msg = utils.createJointTrajectory(self.angles, self.time)
        rospy.loginfo("[ActionMove] Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)
        # rospy.sleep(self.time + 2)


# Class ActionCmdGripping
# The constructor receives a boolean indication whether to grip (True)
# or release (False)
class ActionCmdGripping(Action):
    service_name = "/ariac/gripper/control"

    def __init__(self, toGrip):
        self.toGrip = toGrip

    # This method does not waits untill the gripper status is equals to the desired toGrip
    # See ActionGrippingCheck to check for the gripper status
    def execute_action(self):
        rospy.wait_for_service(ActionGripping.service_name)
        try:
            gripping = rospy.ServiceProxy(
                ActionGripping.service_name, VacuumGripperControl)
            success = gripping(self.toGrip)
            global_vars.partPicked = self.toGrip
            return success

        except rospy.ServiceException as exc:
            rospy.logerr("Failed to use gripper: %s" % exc)
            return False


class ActionGripping(Action):
    service_name = "/ariac/gripper/control"

    def __init__(self, toGrip):
        self.toGrip = toGrip

    def execute_action(self):
        rospy.wait_for_service(ActionGripping.service_name)
        try:
            gripping = rospy.ServiceProxy(
                ActionGripping.service_name, VacuumGripperControl)
            success = gripping(self.toGrip)
            global_vars.partPicked = self.toGrip
            # if not self.toGrip:
            #     rospy.sleep(1)
            count = 1
            while(global_vars.gripper_state.attached is not self.toGrip):
                if count == 100:
                    break
                rospy.sleep(0.05)
                rospy.loginfo(global_vars.gripper_state.attached)
                count += 1

            return success
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to use gripper: %s" % exc)
            return False

class ActionGrippingCheck(Action):
    service_name = "/ariac/gripper/control"

    def __init__(self, toGrip, max_wait, inc_sleep=0.1):
        self.toGrip = toGrip
        self.max_wait = max_wait
        self.inc_sleep = inc_sleep

    def execute_action(self):
        rospy.wait_for_service(ActionGripping.service_name)
        try:

            count = 1
            slept = 0
            rospy.loginfo("[ActionGrippingCheck]: " + str(global_vars.gripper_state.attached))
            success = False
            while(global_vars.gripper_state.attached is not self.toGrip):
                if(slept >= self.max_wait):
                    break
                rospy.sleep(self.inc_sleep)
                rospy.loginfo(global_vars.gripper_state.attached)
                slept += self.inc_sleep

            return global_vars.gripper_state.attached is self.toGrip
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to use gripper: %s" % exc)
            return False

class ActionTurn(Action):

    def __init__(self, object_pos, pose, joint_trajectory_publisher, time, tray_id, material_destination):
        self.object_pos = object_pos
        self.pose = pose
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time
        self.tray_id = tray_id
        self.material_destination = material_destination

    def moveTo(self, angles, speed):
        msg = utils.createJointTrajectory(angles, speed)
        #rospy.loginfo("Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

    def execute_action(self):

        ########## Move to Piece ############
        rospy.loginfo("[ActionTurn] Move to piece")
        result = self.object_pos.calPos
        rotation = self.object_pos.calOri
        position = [result[0], result[1], result[2]]
        # rotation = [orientation[0], orientation[1], orientation[2], orientation[3]]
        # position[0] += 0.05
        # position[1] += 0.05

        rospy.loginfo("[ActionTurn] position= " + str(position))
        # TODO change "pulley_part"
        position_over_piece = ActionMove(position, rotation,
                                         self.joint_trajectory_publisher, 3, "pulley_part", ActionMove.BIN)
        position_over_piece.execute_action()

        rospy.loginfo("## ActionGripping true")
        grapping_action = ActionGripping(True)
        grapping_action.execute_action()

        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(1.5)

        ########## Move UP ############
        rospy.loginfo("[ActionTurn] Move up")

        position = [result[0], result[1], result[2]]
        position[0] -= 0.03
        position[1] -= 0.03
        position[2] += 0.3

        angles = moveUp(position, "pulley_part")
        self.moveTo(angles, 1)
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(1.5)

        ########## Turn Move Right UP ############
        #rospy.loginfo("[ActionTurn] Turnin Move Right")
        rospy.loginfo("[ActionTurn] Turnin Move Right OFF")

        #angles = [2.28, 0.42, -1.0, 3.14, 4.14, -1, 57, 0]
        #self.moveTo(angles, 5)
        #rospy.loginfo("[ActionTurn] SLEEP")
        # rospy.sleep(10)  # DECREASE

        ########## Turn Move Right UP ############
        rospy.loginfo("[ActionTurn] Rotate Gripper")

        angles = [2.28, 0.42, -1.0, 3.14, 4.14, 0.85, 0]
        self.moveTo(angles, 2)
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(2)  # DECREASE

        ########## TGoing Down ############
        rospy.loginfo("[ActionTurn] Going down")

        angles = [2.39, 0.42, -0.38, 3.14, 3.14, 0.88, 0]
        self.moveTo(angles, 1.5)
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(1)  # DECREASE

        ########## Releasing Piece ############
        rospy.loginfo("[ActionTurn] Releasing Piece")

        grapping_action = ActionGripping(False)
        grapping_action.execute_action()
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(1)  # DECREASE

        ########## Going Left ############
        rospy.loginfo("[ActionTurn] Going left")

        angles = [2.39, 0.55, -0.38, 3.14, 3.12, 0.50, 0]
        self.moveTo(angles, 0.5)
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(1)  # DECREASE

        ########## Going up ############
        rospy.loginfo("[ActionTurn] a bit up")
        angles = [2.64, 0.63, -0.758, 3.14, 3.12, -1.47, 0]
        self.moveTo(angles, 0.5)
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(2)  # DECREASE

        ########## Going to Bin7 ############
        rospy.loginfo("[ActionTurn] Going back to bin7")
        angles = STATIC_POSITIONS["bin7"]
        self.moveTo(angles, 0.5)
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(1)  # DECREASE

        ######### GET POS of piece #######
        # calculating object position according to camera
        rospy.loginfo("[ActionTurn] kit_object = " +
                      str(self.object_pos.kit_object))
        self.object_pos = ActionCameraTFTurn(
            self.object_pos.kit_object, self.object_pos.part_name)

        self.object_pos.execute_action()
        rospy.loginfo("[ActionTurn] new pos = " +
                      str(self.object_pos.calPos) + str(self.object_pos.calOri))
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(2)  # DECREASE

        ########## Move to Piece ############
        rospy.loginfo("[ActionTurn] Move to piece")
        result = self.object_pos.calPos
        rotation = self.object_pos.calOri
        position = [result[0], result[1], result[2]]
        # rotation = [orientation[0], orientation[1], orientation[2], orientation[3]]
        # the object has a different orientation now
        position[2] -= (OBJECT_HEIGHT["PULLEY_PART"] + 0.01)

        # TODO change "pulley_part"
        position_over_piece = ActionMove(position, rotation,
                                         self.joint_trajectory_publisher, 3, "pulley_part_turn", ActionMove.BIN)
        position_over_piece.execute_action()

        rospy.loginfo("## ActionGripping true")
        grapping_action = ActionGripping(True)
        grapping_action.execute_action()

        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(1)

        ########## Move to init ############
        rospy.loginfo("[ActionTurn] Move to init")
        initial_position_bin = ActionPose(
            STATIC_POSITIONS["bin7"], self.joint_trajectory_publisher, 3.0)
        initial_position_bin.execute_action()
        rospy.loginfo("[ActionTurn] SLEEP")
        rospy.sleep(3.0)


class ActionTurnInBarYellow(Action):

    def __init__(self, object_pos, pose, joint_trajectory_publisher, time, tray_id, material_destination):
        self.object_pos = object_pos
        self.pose = pose
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time
        self.tray_id = tray_id
        self.material_destination = material_destination

    def moveTo(self, angles, speed):
        msg = utils.createJointTrajectory(angles, speed)
        #rospy.loginfo("Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)
        time.sleep(0.1)

    def execute_action(self):

        ########## Move to Piece ############
        rospy.loginfo("[ActionTurn] Move to piece")
        result = self.object_pos.calPos
        orientation = self.object_pos.calOri
        position = [result[0], result[1], result[2]]
        position[0] += 0.03
        position[1] += 0.03

        rospy.loginfo("[ActionTurn] position= " + str(position))
        # TODO change "pulley_part"
        position_over_piece = ActionMove(position, orientation,
                                         self.joint_trajectory_publisher, 3, "pulley_part", ActionMove.BIN)
        position_over_piece.execute_action()

        rospy.loginfo("## ActionGripping true")
        grapping_action = ActionGripping(True)
        grapping_action.execute_action()

        rospy.loginfo("[ActionTurn] SLEEP")
        time.sleep(5.0)

        ########## Move UP ############
        rospy.loginfo("[ActionTurn] Move up")

        position = [result[0], result[1], result[2]]
        position[0] -= 0.03
        position[1] -= 0.03
        position[2] += 0.3

        angles = moveUp(position, "pulley_part")
        self.moveTo(angles, 1)
        rospy.loginfo("[ActionTurn] SLEEP")
        time.sleep(5.0)

        #[0.34, 0.93, 2.02]
        ########## Turn Move Right UP ############
        #rospy.loginfo("[ActionTurn] Turning Move Right")
        rospy.loginfo("[ActionTurn] Turning Move Right OFF")

        angles = STATIC_POSITIONS["bin8"]
        self.moveTo(angles, 3)
        teste = ActionSleep(3, "Bin8")
        teste.execute_action()

        angles = [2.62, 1.15, -1.80, 3.14, 3.34, -1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Move Up 1")
        teste.execute_action()

        angles = [2.62, 1.15, -2.18, 3.14, 3.34, 1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Move Up 2")
        teste.execute_action()

        angles = [1.61, 1.15, -2.18, 3.14, 5.03, 1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Move Up 3")
        teste.execute_action()

        angles = [1.44, 1.15, -1.79, 3.14, 5.03, 1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Place 1")
        teste.execute_action()

        grapping_action = ActionGripping(False)
        grapping_action.execute_action()
        teste = ActionSleep(5, "Drop")
        teste.execute_action()

        angles = [1.50, 1.15, -1.81, 3.14, 5.03, 1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(1, "Out 1")
        teste.execute_action()

        angles = [1.75, 1.15, -2.19, 3.14, 5.03, 1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(1, "Out 2")
        teste.execute_action()

        angles = [1.00, 1.15, -1.90, 3.14, 5.03, -1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(1, "Out 3")
        teste.execute_action()

        angles = [1.00, 1.15, -1.90, 3.14, 5.03, -1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Adjust 1")
        teste.execute_action()

        angles = [0.50, 1.15, -1.50, 3.14, 5.60, -1.57, 0]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Adjust 2")
        teste.execute_action()

        #angles = [0.10, 1.18, -1.19, 3.14, 5.80, -1.57, 0]
        #self.moveTo(angles, 4)
        #teste = ActionSleep(3, "Take")
        # teste.execute_action()

        angles = [-0.10, 1.18, -1.08, 3.14, 5.85, -1.57, 0]
        self.moveTo(angles, 4)
        teste = ActionSleep(3, "Take TEST")
        teste.execute_action()

        grapping_action = ActionGripping(True)
        grapping_action.execute_action()
        teste = ActionSleep(5, "Take command")
        teste.execute_action()

        angles = [0.11, 1.18, -1.24, 3.14, 5.68, -1.57, 1.57 - rotation[2]]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Out with Pulley")
        teste.execute_action()

        angles = [1.49, 1.18, -2.25, 3.14, 5.18, -1.57, 1.57 - rotation[2]]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Adjust 1")
        teste.execute_action()

        angles = [2.12, 1.18, -2.12, 3.14, 4.30, -1.57, 1.57 - rotation[2]]
        self.moveTo(angles, 1)
        teste = ActionSleep(3, "Adjust 2")
        teste.execute_action()


class ActionTurnSameBin(Action):

    def __init__(self, actionTF, kit_object, joint_trajectory_publisher, gprt, time):
        self.actionTF = actionTF
        self.kit_object = kit_object
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.time = time
        self.gprt = gprt

    def moveTo(self, angles, speed):
        msg = utils.createJointTrajectory(angles, speed)
        #rospy.loginfo("Sending command:\n" + str(msg))
        self.joint_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

    def moveSideways(self, distance, timeToGoal=2):
        currPos = self.gprt.current_joint_state.position
        newAngles = []
        newAngles.extend(currPos)
        newAngles[1] = newAngles[1] + distance

        actionPose = ActionPose(
            newAngles, self.joint_trajectory_publisher, timeToGoal)
        actionPose.execute_action()

        actCheckGoal = ActionGoal(msg="[ActionTurnSameBin] Waiting while going sideway " + str(distance) + "cm",
                                      gprt=self.gprt, action=actionPose, max_sleep=2)
        actCheckGoal.execute_action()

    def moveToolTip(self, distanceZ, distanceX, timeToGoal=5):
        moveToolTip = MoveUpToolTip(
            self.gprt, self.joint_trajectory_publisher, incrementZ=distanceZ, incrementX=distanceX, controlWr1=True, timeToGoal=timeToGoal)

        moveToolTip.execute_action()

        actCheckGoal = ActionGoal(msg="Waiting while moving tooltip z=" + str(distanceZ) + "cm " + "x = " + str(distanceX),
                                      gprt=self.gprt, action=moveToolTip, max_sleep=timeToGoal + 2)
        actCheckGoal.execute_action()


    def waitForGripper(self, cmdSent, toGrip, inc_sleep=0.1):
        if(cmdSent):  # If the command could be executed
            rospy.logerr(
                "[ActionTurnSameBin] - Waiting while gripper state is " + str(toGrip))
            checkGripperState = ActionGrippingCheck(
                toGrip=toGrip, max_wait=5, inc_sleep=inc_sleep)
            sucess = checkGripperState.execute_action()
        else:
            rospy.logerr("[ActionTurnSameBin] - Severe ERROR.\n" +
                         "We were not able to send the command to the gripper control service")

        if(not sucess):  # Sucess mean the command was executed
            rospy.logerr("[ActionTurnSameBin] - Severe ERROR.\n" +
                         "We should have attached the gripper already")
            # TODO - Do something. Like adding action back

    def goToPiece(self, waitReach=False, ignoreHeight=False):
        rospy.loginfo("[ActionTurnSameBin] goToPiece")
        position_over_piece = ActionMovePiece(
            self.actionTF, self.joint_trajectory_publisher, 1.5, self.kit_object.type, ActionMove.BIN, ignoreHeight=ignoreHeight)
        position_over_piece.execute_action()

        if(waitReach):
            actCheckGoal = ActionGoal(msg="[ActionTurnSameBin] Waiting while reaching piece, pos:  " + str(self.actionTF.calPos),
                                      gprt=self.gprt, action=position_over_piece, max_sleep=2)
            actCheckGoal.execute_action()

    def goToABitAbovePiece(self, waitReach=False, ignoreHeight=False):
        rospy.loginfo("[ActionTurnSameBin] goToABitAbovePiece - waitReach: " +
                      str(waitReach) + "ignoreHeight: " + str(ignoreHeight))
        position_over_piece = ActionMoveABitAbovePiece(
            self.actionTF, self.joint_trajectory_publisher, 1.5, self.kit_object.type, ActionMove.BIN, ignoreHeight=ignoreHeight, distance=0.008)
        position_over_piece.execute_action()

        if(waitReach):
            actCheckGoal = ActionGoal(msg="[ActionTurnSameBin] goToABitAbovePiece waitReach pos:  " + str(self.actionTF.calPos),
                                      gprt=self.gprt, action=position_over_piece, max_sleep=2)
            actCheckGoal.execute_action()

    def turnWrit2(self, angle):
        turnWrit2 = ActionTurnWrist2(angle=angle,
                                     joint_trajectory_publisher=self.joint_trajectory_publisher,
                                     gprt=self.gprt, time=0.8)
        angles_goal = turnWrit2.execute_action()

        actCheckGoal = ActionGoal(msg="[ActionTurnSameBin] Waiting while reaching turnWrit2: " + str(angle) + "rad",
                                      gprt=self.gprt, action=turnWrit2, max_sleep=0.9)
        actCheckGoal.execute_action()

    def goDownUntilGetPiece(self, ignoreHeight=False):
        rospy.loginfo("[ActionTurnSameBin] goDownUntilGetPiece")
        tfPos = self.actionTF.calPos
        tfOri = self.actionTF.calOri
        position = [tfPos[0], tfPos[1], tfPos[2] - 0.02]

        # rospy.sleep(self.time + 2) #DECREASE
        rospy.loginfo("[ActionTurnSameBin] goDownUntilGetPiece pos = " +
                      str(position) + "  self = " + str(self))
        angles = solverBin(
            position, tfOri, self.kit_object.type, ignoreHeight=ignoreHeight)

        position2 = [tfPos[0], tfPos[1], tfPos[2] + 0.02]
        angles2 = solverBin(
            position2, tfOri, self.kit_object.type, ignoreHeight=ignoreHeight)

        msg = utils.createJointTrajectory(angles, 3)
        msg2 = utils.createJointTrajectory(angles2, 1)
        self.joint_trajectory_publisher.publish(msg)
        sendCmdToGripper = ActionCmdGripping(toGrip=True)
        cmdSent = sendCmdToGripper.execute_action()
        # Wait while the gripper attaches
        # returns as soon as the gripper attaches
        self.waitForGripper(cmdSent=cmdSent, toGrip=True, inc_sleep=0.01)
        self.joint_trajectory_publisher.publish(msg2)
        rospy.sleep(0.1)
        


    def execute_action(self):

        calPos = self.actionTF.calPos

        # Move to Piece
        self.goToABitAbovePiece(waitReach=True, ignoreHeight=False)
        
        sendCmdToGripper = ActionCmdGripping(toGrip=True)
        cmdSent = sendCmdToGripper.execute_action()

        # #Be aware of decreasing this time
        # actSleep = ActionSleep(time=10, msg="[ActionTurnSameBin]  Debug Sleep")
        # actSleep.execute_action()


        self.goDownUntilGetPiece()

        # Move ToolTip UP
        self.moveToolTip(distanceZ=0.2, distanceX=0.1, timeToGoal=1.4)

        # Turn Wrist2
        self.turnWrit2(0.01)

        # Move a bit to the side
        self.moveSideways(0.022, timeToGoal=1)

        # Move ToolTip close to gaol really fast
        self.moveToolTip(distanceZ=-0.168, distanceX=0.13, timeToGoal=2)
        # actSleep = ActionSleep(time=10, msg="[ActionTurnSameBin]  Debug Sleep")
        # actSleep.execute_action()

        # Move ToolTip to the goal
        # self.moveToolTip(distance=-0.01, timeToGoal=1.9)

        # actSleep = ActionSleep(time=10, msg="[ActionTurnSameBin]  Debug Sleep")
        # actSleep.execute_action()

        # Be aware of decreasing this time
        rospy.sleep(0.7)  # This avoids dropping the piece too fast

        sendCmdToGripper = ActionCmdGripping(toGrip=False)
        cmdSent = sendCmdToGripper.execute_action()

        # Move a bit to the other side
        self.moveSideways(0.04, timeToGoal=0.3)

        # Wait while the gripper detaches
        # returns as soon as the gripper attaches
        self.waitForGripper(cmdSent=cmdSent, toGrip=False)

        # Turn Wrist2
        self.turnWrit2(-1.5707963268)

        
        # Move ToolTip UP
        self.moveToolTip(distanceZ=0.4, distanceX=0, timeToGoal=0.3) 

        # Move a bit to the other side
        self.moveSideways(-0.4, timeToGoal=0.1)

        # Move ToolTip Down
        self.moveToolTip(-0.01, distanceX=0.23, timeToGoal=0.2)


        # Move a bit to the other side
        self.moveSideways(0.3, timeToGoal=3)
        # sys.exit(0)

        rospy.sleep(0.4)  # time to update tf

        self.actionTF = ActionCameraTFTurn( ####################
            self.actionTF.kit_object, self.actionTF.part_name)

        self.actionTF.execute_action()


        # Be aware of decreasing this time
        actSleep = ActionSleep(time=1, msg="[ActionTurnSameBin]  Debug Sleep")
        actSleep.execute_action()

        # Move to Piece
        self.goToABitAbovePiece(waitReach=True, ignoreHeight=True)

        sendCmdToGripper = ActionCmdGripping(toGrip=True)
        cmdSent = sendCmdToGripper.execute_action()

        self.goDownUntilGetPiece(ignoreHeight=True)

        # Move ToolTip UP
        self.moveToolTip(distanceZ=0.3, distanceX=0.1, timeToGoal=0.4)




class ActionSendAgv(Action):
    service_name = "/ariac/agv{0}"

    def __init__(self, agv_id, kit_type):
        # id must be the number of the agv
        self.agv_id = agv_id
        # kit_type from the order
        self.kit_type = kit_type

    def execute_action(self):
        agvServiceName = ActionSendAgv.service_name.format(self.agv_id)
        rospy.loginfo("sending 'kit_type: " + self.kit_type +
                      " to : " + agvServiceName)
        rospy.wait_for_service(agvServiceName)
        try:

            send_agv = rospy.ServiceProxy(
                agvServiceName, AGVControl)
            success = send_agv(self.kit_type)
            rospy.sleep(1)
            return success
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to notify agv %s: %s" % (self.kit_type, exc))
            return False


class ActionCheckFaulty(Action):
    service_name = "/ariac/quality_control_sensor_{0}"

    def __init__(self, agv_id):
        self.agv_id = agv_id

    def execute_action(self):
        faultyCheckServiceName = ActionCheckFaulty.service_name.format(
            self.agv_id)
        rospy.wait_for_service(faultyCheckServiceName)
        try:
            check = rospy.ServiceProxy(
                faultyCheckServiceName, LogicalCameraImage)
            success = send_agv(self.kit_type)
            return success
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to check faulty at agv %s: %s" %
                         (self.agv_id, exc))
            return False


class ActionBeltControl(Action):
    service_name = "/ariac/conveyor/control"

    def __init__(self, toMove):
        self.toMove = toMove

    def execute_action(self):
        rospy.wait_for_service(self.service_name)
        try:
            control_belt = rospy.ServiceProxy(
                self.service_name, ConveyorBeltControl)
            beltState = ConveyorBeltState()
            beltState.power = self.toMove
            success = control_belt(beltState)
            return success
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to control belt: %s" % exc)
            return False

class ActionSleep(Action):

    def __init__(self, time, msg):
        self.msg = msg
        self.time = time
       

    def execute_action(self):

        rospy.loginfo("[ActionSleep]- sleeping " +
                      str(self.time) + "s. \nReason: " + self.msg)
        rospy.sleep(self.time)


class ActionTurnWrist2(Action):

    def __init__(self, angle, joint_trajectory_publisher, gprt, time):
        self.angle = angle
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.gprt = gprt
        self.time = time

       

    def execute_action(self):

        position = self.gprt.current_joint_state.position
        self.angles = []
        self.angles.extend(position)
        rospy.loginfo("[ActionTurnWrist2] - Moving Writ2 to" + str(self.angle))
        self.angles[5] = self.angle

        actionPose = ActionPose(
            self.angles, self.joint_trajectory_publisher, self.time)
        actionPose.execute_action()

        return self.angles


class ActionGoal(Action):

    def __init__(self, msg, gprt, position=None,
                 action=None, max_sleep=8,
                 accError=[0.009, 0.009, 0.009, 0.009, 0.009, 0.01, 0.009, 0.009, 0.009], tray_id=None, kit_type = None, kit_object=None):
        self.max_sleep = max_sleep
        self.inc_sleep = 0.1
        self.msg = msg
        self.gprt = gprt
        self.kit_type = kit_type
        self.kit_object = kit_object
        self.position = position
        self.action = action
        self.tray_id = tray_id
        self.accError = accError

    def execute_action(self):
        rospy.logerr("[ActionGoal] - " + self.msg)
        if self.position is not None or self.action is not None:
            if self.action is not None:
                self.position = self.action.angles
            result = False
            slept = 0
            while not result and slept < self.max_sleep:
                position = self.gprt.current_joint_state.position
                result, listRest = utils.comparePosition(
                    position, self.position, self.accError)
                if(not result):
                    rospy.sleep(self.inc_sleep)
                    slept += self.inc_sleep

            if(not result):
                rospy.logerr("[ActionGoal] - Goal not reached")
                rospy.logerr("[ActionGoal] - List Joint: " + str(listRest))
                rospy.logerr("[ActionGoal] - Expected position " +
                             str(self.position) + " | Actual position " + str(position))
            else:
                rospy.loginfo("[ActionGoal] - Goal Reached")

        else:
            rospy.logerr("[ActionGoalDebug] - Deprecated - Cannot guarantee behaviour- " + self.msg)

        if global_vars.partPicked:
            grippingCheck_action = ActionGrippingCheck(toGrip=True, max_wait=5)
            checkGripperDropped=grippingCheck_action.execute_action()
            print ("\n\n\n\n " + str(self.kit_type) + " " + str(self.kit_object) + " \n\n\n\n\n")
            
            if (not checkGripperDropped and self.kit_type is not None and self.kit_object is not None and not global_vars.CheckTaskIsFinished):

                #rospy.logerr("[ActionCheckDrop] - ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n")
                rospy.logerr("[ActionCheckDrop] - Adding Action Back")
                action_tray = self.gprt.actions_tray1 if self.tray_id == 1 else self.gprt.actions_tray2
                object_type = self.kit_object.type
                rospy.loginfo(
                    "[ActionCheckDrop] Adding material check for object " + str(object_type))
                checkAction = ActionMaterialLocation(
                    object_type, self.kit_object, self.kit_type, "Recovery Material")
                action_tray.insert(0, checkAction)
                global_vars.CheckTaskIsFinished = True


class ActionCheckAndDiscardPiece(Action):

    def __init__(self, gprt, joint_trajectory_publisher, tray_id, kit_object, action, actionsList, comp_class, last):
        self.gprt = gprt
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.tray_id = tray_id
        self.kit_object = kit_object
        self.action = action
        self.actionsList = actionsList
        self.comp_class = comp_class
        self.last = last

    def execute_action(self):
        rospy.logerr(
            "\n\n\n[ActionCheckAndDiscardPiece] - LAST VALUE: " + str(self.last) + "\n\n\n")

        faulty_sensor_msg = 0
        near_agv = 0
        actMov_agv = 0
        if(self.tray_id == 1):
            faulty_sensor_msg = self.gprt.faulty_sensor1
            near_agv = STATIC_POSITIONS["agv1"]
            actMov_agv = ActionMove.AGV1
            disBelAgvAct = ActionPose(
                STATIC_POSITIONS["disBelAgv1"], self.joint_trajectory_publisher, 1.0)
            gambiPos = STATIC_POSITIONS["bin8"]
        else:
            faulty_sensor_msg = self.gprt.faulty_sensor2
            near_agv = STATIC_POSITIONS["agv2"]
            actMov_agv = ActionMove.AGV2
            disBelAgvAct = ActionPose(
                STATIC_POSITIONS["disBelAgv2"], self.joint_trajectory_publisher, 3.0)
            gambiPos = STATIC_POSITIONS["bin5"]

        falty = len(faulty_sensor_msg) > 0

        if falty:
            rospy.logerr("[ActionCheckAndDiscardPiece] - Falty Piece Detected")

            # rospy.sleep(1)

            movepos, rotapos = utils.calculate_order_position(
                self.kit_object.pose, self.tray_id, incrementZ=-0.01)
            piece_to_tray = ActionMove(
                movepos, rotapos, self.joint_trajectory_publisher, 0.3, self.kit_object.type, actMov_agv)
            rospy.logerr("[ActionCheckAndDiscardPiece] - Move to Piece")
            piece_to_tray.execute_action()


            rospy.logerr("[ActionCheckAndDiscardPiece] - Pick Piece")
            sendCmdToGripper = ActionCmdGripping(toGrip=True)
            cmdSent = sendCmdToGripper.execute_action()

            if(cmdSent):  # If the command could be executed
                rospy.logerr(
                    "[ActionCheckAndDiscardPiece] - Waiting while gripper state is True")
                checkGripperState = ActionGrippingCheck(
                    toGrip=True, max_wait=5)
                sucess = checkGripperState.execute_action()
            else:
                rospy.logerr("[ActionCheckAndDiscardPiece] - Severe ERROR.\n" +
                             "We were not able to send the command to the gripper control service")

            if(not sucess):  # Sucess mean the command was executed
                rospy.logerr("[ActionCheckAndDiscardPiece] - Severe ERROR.\n" +
                             "We should have attached the gripper already")
                # TODO - Do something. Like adding action back

            # rospy.sleep(2)

            near_agvAct = ActionPose(
                near_agv, self.joint_trajectory_publisher, 1.0)
            rospy.logerr("[ActionCheckAndDiscardPiece] - Move to AGV Start")
            near_agvAct.execute_action()

            actCheckGoal = ActionGoal(msg="Waiting while reaching AGV Start", 
                                      gprt=self.gprt, action=near_agvAct, max_sleep=8)
            actCheckGoal.execute_action()

            rospy.logerr(
                "[ActionCheckAndDiscardPiece] - Move to Belt to discard")
            disBelAgvAct.execute_action()


            actCheckGoal = ActionGoal(msg="Waiting while reaching BelAgvAct", 
                                      gprt=self.gprt, action=disBelAgvAct, max_sleep=7)
            actCheckGoal.execute_action()


            rospy.logerr("[ActionCheckAndDiscardPiece] - Drop Piece")
            sendCmdToGripper = ActionCmdGripping(toGrip=False)
            cmdSent = sendCmdToGripper.execute_action()

            if(cmdSent): #If the command could be executed   
                rospy.logerr("[ActionCheckAndDiscardPiece] - Waiting while gripper state is False")                        
                checkGripperState = ActionGrippingCheck(toGrip=False, max_wait=5)
                sucess = checkGripperState.execute_action()
            else:
                rospy.logerr("[ActionCheckAndDiscardPiece] - Severe ERROR.\n"+
                             "We were not able to send the command to the gripper control service")    

            if(not sucess): #Sucess mean the piece was dropped
                rospy.logerr("[ActionCheckAndDiscardPiece] - Severe ERROR.\n"+
                             "We should have dropped the piece already")
                #TODO - Do something. Like adding action back


            rospy.logerr("[ActionCheckAndDiscardPiece] - Adding Action Back")

            action_tray = self.gprt.actions_tray1 if self.tray_id == 1 else self.gprt.actions_tray2
            # rospy.loginfo("[ActionCheckAndDiscardPiece] len(action_tray) = " + str(len(action_tray)))
            # rospy.loginfo("[ActionCheckAndDiscardPiece] action_tray = " + str(action_tray))
            object_type = self.action.kit_object.type
            rospy.loginfo(
                "[ActionCheckAndDiscardPiece] Adding material check for object " + str(object_type))
            checkAction = ActionMaterialLocation(
                object_type, self.action.kit_object, self.action.kit_type, "Recovery Material")
            #action_tray.insert(len(action_tray) - 1, checkAction)
            action_tray.insert(0, checkAction)

        else:
            rospy.logerr("[ActionCheckAndDiscardPiece] - Piece OK")


            near_agvAct = ActionPose(
                near_agv, self.joint_trajectory_publisher, 0.2)
            rospy.logerr("[ActionCheckAndDiscardPiece] - Move to near_agvAct")
            near_agvAct.execute_action()

            actCheckGoal = ActionGoal(msg="Waiting while reaching near_agvAct", 
                                      gprt=self.gprt, action=near_agvAct, max_sleep=8)
            actCheckGoal.execute_action()

            if(self.tray_id == 1 and len(self.comp_class.actions_tray1) == 0):
                rospy.loginfo(
                    "[ActionCheckAndDiscardPiece] - Send Order - Tray 1")
                send_agv = ActionSendAgv(1, self.action.kit_type)
                send_agv.execute_action()
            elif(self.tray_id == 2 and len(self.comp_class.actions_tray2) == 0):
                rospy.loginfo(
                    "[ActionCheckAndDiscardPiece] - Send Order - Tray 2")
                send_agv = ActionSendAgv(2, self.action.kit_type)
                send_agv.execute_action()
            else:
                rospy.logerr(
                    "[ActionCheckAndDiscardPiece] - Unexpected results. More Trays?")

            binPos = ActionPose(
                gambiPos, self.joint_trajectory_publisher, 0.5)
            rospy.logerr("[ActionCheckAndDiscardPiece] - Move to BIN Start")
            binPos.execute_action()

            actCheckGoal = ActionGoal(msg="Waiting while reaching BIN Start", 
                                      gprt=self.gprt, action=binPos, max_sleep=8)
            actCheckGoal.execute_action()


        rospy.logerr("[ActionCheckAndDiscardPiece]  DONE")

class MoveUpToolTip(Action):

    def __init__(self, gprt, joint_trajectory_publisher, incrementZ=0.3, incrementX=0.1, controlWr1=True, timeToGoal=0.2):
        self.gprt = gprt
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.incrementZ = incrementZ
        self.incrementX = incrementX
        self.controlWr1 = controlWr1
        self.timeToGoal = timeToGoal

    def moveTo(self, angles, speed):
        msg = utils.createJointTrajectory(angles, speed)
        self.joint_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

    def execute_action(self):
        ########REMOVE ##########
        move = True
        controlWr1 = self.controlWr1

        posUpperArm, angleUpperArm = utils.getUpperArmPose()
        posVacum, angleVacum = utils.getVacuumGripperPos()
        posFore, angleFore = utils.getForeArmPos()

        workingPos = deepcopy(posVacum)
        workingPos[2] += self.incrementZ
        workingPos[0] += self.incrementX


        shoulderBTriangle = utils.computeXZDistance(posUpperArm, workingPos)

        xDistance = abs(utils.computeXDistance(posUpperArm, workingPos))
        zDistance = abs(utils.computeZDistance(posUpperArm, workingPos))


        wristBTriangle = utils.computeXZDistance(workingPos, posVacum)


        # sameZLevel = zDistance is 0 #not handling yet
        workingIsAbove = utils.computeZDistance(posUpperArm, workingPos) < 0

        rospy.loginfo("posUpperArm = " + str(posUpperArm))
        rospy.loginfo("posVacum = " + str(posVacum))
        rospy.loginfo("posFore = " + str(posFore))
        rospy.loginfo("angleFore = " + str(angleFore))
        rospy.loginfo("workingPos = " + str(workingPos))
        rospy.loginfo("Base Triangle = " + str(shoulderBTriangle))
        rospy.loginfo("xDistance = " + str(xDistance))
        rospy.loginfo("zDistance = " + str(zDistance))


        a1, b1, c1, A1, B1, C1 = solve(
            a=shoulderBTriangle, b=FORE_ARM, c=UP_ARM)

        a2, b2, c2, A2, B2, C2 = solve(
            a=xDistance, b=zDistance, c=shoulderBTriangle)

        elbow_joint = math.pi - A1

        shoulder_lift_joint = math.pi / 2 - A2 -B1
        wrist_1_joint =  math.pi - (C1 + B2) + math.pi/2


        rospy.loginfo("#A1,B1,C1 = " + str(A1) +
                      ", " + str(B1) + ", " + str(C1))

        rospy.loginfo("#A2,B2,C2 = " + str(A2) +
                      ", " + str(B2) + ", " + str(C2))
     
        self.angles = []
        self.angles.extend(self.gprt.current_joint_state.position)
        self.angles[0] = elbow_joint
        self.angles[2] = shoulder_lift_joint
        self.angles[4] = wrist_1_joint

        pauseToDrop = ActionSleep(1, "Pause before Drop")
        pauseToDrop.execute_action()


        if(move):
            rospy.loginfo("\n\n\nMOVING UP TO =" + str(self.angles))
            self.moveTo(self.angles, self.timeToGoal)
        