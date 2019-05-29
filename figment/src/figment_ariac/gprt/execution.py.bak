#!/usr/bin/env python
import arm_actions
import global_vars
import rospy
import transform
import sys

from utils import *
from constants import *
from ik_solution import solverBelt
from trajectory_msgs.msg import JointTrajectory
import gripper_actions

from osrf_gear.srv import AGVControl


class ExecBelt:

    def __init__(self, part_plan, exec_part):
        self.part_plan = part_plan
        self.exec_part = exec_part

    def execute(self):

        exec_step = 0
        failed_comple = False
        done = False

###################       STEP 0       ###################################

        while(not failed_comple and not done and not self.exec_part.isInterupted()):
            jump = False

            if(not jump and exec_step <= 0):  # STEP 0 - Setup env
                part_origin = self.part_plan.pick_piece.origin.value
                part_type = self.part_plan.part.part_type
                part_world_position = self.part_plan.pick_piece.world_position
                part_world_orientation = self.part_plan.pick_piece.world_orientation
                tray_id = self.part_plan.dest_tray_id
                desired_part_pose = self.part_plan.part.desired_pose
                exec_step = +1  # STEP 0 - DONE
                gripper_actions.send_gripping_cmd_and_wait(False)
                to_flip = self.part_plan.to_flip
                if to_flip:
                    rospy.loginfo("\n\n[ExecBelt][ExecutePart]: STEP 0 : Part must be FLIPPED\n\n\n\n")

###################       STEP 1       ###################################
            if(not jump and exec_step <= 1 and not self.exec_part.isInterupted()):  # STEP 1 - Check Belt

                if part_origin == PickPlaces.BELT.value:
                    rospy.loginfo("\n\n[ExecuteBeltPart]: STEP 1 \n")
                    # step 0 - get available pose for a part on any bin
                    camera_name = ORIGN_CAMERA["belt"] + "_frame"
                    camera_id, part_id = global_vars.tf_manager.find_part_name(
                        part_type, sub_dad=camera_name)
                    part_name = part_id[part_id.find(part_type):-6]
                    if(camera_id is None or part_id is None):
                        rospy.loginfo(
                            "[ExecuteBeltPart]:Failed. No available part {} found".format(part_type))
                        self.part_plan.part.reset()
                        return False
                    r = self.exec_part.find_part_any_belt(
                        camera_id, part_id, part_type)

                    if(r is None):
                        rospy.loginfo(
                            "[ExecuteBeltPart]:Failed. No available part {} found".format(part_type))
                        self.part_plan.part.reset()
                        return False
                    part_world_position, part_world_orientation, part_world_tf_time = r

                    exec_step = +1  # STEP 1 - DONE

###################       STEP 2       ###################################

            # STEP 2 - move to belt static position
            if(not jump and exec_step <= 2 and not self.exec_part.isInterupted()):

                rospy.loginfo("\n\n[ExecuteBeltPart]: STEP 2 \n")
                # step 1 - move to position in front of the piece
                success = self.exec_part.move_wait_belt(part_world_position)
                if not success:
                    rospy.loginfo("[ExecuteBeltPart]: step2 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step = +1  # STEP 2 - DONE

###################       STEP 3       ###################################

            # STEP 3 - follow the part on the belt
            if(not jump and exec_step <= 3 and not self.exec_part.isInterupted()):

                rospy.loginfo("\n\n[ExecuteBeltPart]: STEP 3 \n")
                max_attempt = 1
                attempt = 0
                success = False

                # if part is a pulley and should be flipped 
                # we will get it at the tip of the part
                if to_flip:
                    part_world_position[0] -= 0.08
                    # part_world_position[1] += 0.03


                while(attempt < max_attempt and not success):
                    success = self.exec_part.move_towards_piece_on_belt(part_world_position,
                                                                        part_world_orientation,
                                                                        part_world_tf_time, part_type)
                    if not success:
                        attempt += 1
                        rospy.loginfo(
                            "[ExecuteBeltPart]: step3 failed. attempt#" + str(attempt))

                if not success:
                    rospy.loginfo(
                        "[ExecuteBeltPart]: step3 failed completly" + str(attempt))
                    gripper_actions.send_gripping_cmd_and_wait(False)
                    global_vars.tf_manager.add_part_id_to_bl(part_id)
                    self.part_plan.part.reset()
                    return False

                exec_step = +1  # STEP 3 - DONE

###################       STEP 4       ###################################

            if(not jump and exec_step <= 4 and not self.exec_part.isInterupted()): #STEP 4 - Move To TRAY
                
                # part must be flipped
                if to_flip:
                    self.exec_part.turn_pulley_yellow_bar(part_world_orientation)


                rospy.loginfo("\n\n[ExecuteBeltPart]: STEP 4 \n")
                success = self.exec_part.move_to_tray(tray_id)
                # success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecuteBeltPart]: step5 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP  - DONE

###################       STEP 5       ##########################################  
#STEP 5 - Put Part at tray              
            if(not jump and exec_step <= 5 and not self.exec_part.isInterupted()): 
                rospy.loginfo("\n\n[ExecuteBeltPart]: STEP 5 \n")
                
                if(self.part_plan.dest_tray_id == 1):
                    camera_name = AGVS_CAMERA["agv1"]
                    solver = arm_actions.SolverType.AGV1
                    incrementY=-0.1
                elif(self.part_plan.dest_tray_id == 2): 
                    camera_name = AGVS_CAMERA["agv2"]
                    solver = arm_actions.SolverType.AGV2
                    incrementY=0.1
                success = self.exec_part.deposit_at_tray(desired_part_pose=desired_part_pose, part_type=part_type, tray_id=tray_id, force_check_piece=True, time_to_execute_action=1, accError=[0.009, 0.009, 0.009, 0.009,0.015, 0.015, 0.009, 0.009, 0.1], flipped_part = self.part_plan.to_flip)
                rospy.loginfo("[ExecuteBeltPart]: step5 solver = {}".format(solver))
                if not success:
                    rospy.loginfo("[ExecuteBeltPart]: step5 failed. Reseting")
                    rospy.loginfo("[ExecuteBeltPart][STEP5] - Move tool tip up")
                    arm_actions.moveToolTipZY(incrementZ=0.2, incrementY=incrementY, timeToGoal=0.3)

                    # waiting tf_manager update
                    camera_id, part_id = global_vars.tf_manager.find_part_name(part_name=part_name, sub_dad=camera_name)
                    rospy.loginfo("[ExecuteBeltPart]: DEBUG camera_id: {} ; part_id{}".format(camera_id, part_id))
                    if(len(camera_id) == 0 or len(part_id) == 0): #part not found
                        rospy.loginfo("[ExecuteBeltPart]: step7 failed [part not found]. Reseting")
                        # arm_actions.moveToolTipZY(incrementZ=0.2, incrementY=incrementY, timeToGoal=0.2)
                        success = self.exec_part.move_to_tray(tray_id=tray_id, 
                                                    force_check_piece=False, 
                                                    force_grp_sts=False, 
                                                    time=1)
                        # rospy.loginfo("[ExecutePart]: DEBUG SLEEL \n\n\n")
                        # rospy.sleep(10)
                        if not success:
                            #TODO MOVE UP A BIT
                            rospy.logerr("[ExecuteBeltPart]: step5 failed. Could not get back to AGV")
                        self.part_plan.part.reset()
                        return False    
                    # rospy.sleep(1)
                    r = self.exec_part.find_part_any_agvs(part_id)#TODO any agv or a specific agv?
                    
                    if r is not None:
                        part_world_position, part_world_orientation = r

                        #print ("_____________________________________________________________________________")
                        print ("\n\n\n\n " + str(part_world_position) + " \n\n\n\n\n")
                        print ("\n\n\n\n " + str(part_world_orientation) + " \n\n\n\n\n")
                        print ("\n\n\n\n " + str(part_type) + " \n\n\n\n\n") 
                        
                        # success = self.exec_part.move_wait_above_part(part_world_position, part_world_orientation, part_type, solver, 0.2)

                        rospy.loginfo("[ExecuteBeltPart][STEP5] - Move Wait a bit above")
                        success = self.exec_part.move_wait_above_part(part_world_position=part_world_position, 
                                                part_world_orientation=part_world_orientation, 
                                                part_type=part_type, solver_type=solver, 
                                                a_bit_above_value=0.05, 
                                                time_to_execute_action=1)
                        # rospy.sleep(10)
                        if success:
                            rospy.loginfo("[ExecuteBeltPart][STEP5] - go_down_until_get_piece")
                            success = arm_actions.go_down_until_get_piece(world_position=part_world_position, 
                                                                    world_orientation=part_world_orientation, 
                                                                    part_type=part_type, 
                                                                    time=10, ignore_height=False, 
                                                                    distance=0.005, solver_type=solver,
                                                                    adjust=True)
                        
                            arm_actions.moveToolTipZY(0.3, incrementY, 1.4)

                            rospy.logerr("........................................................................")
                            if success :
                                rospy.logerr("....................SUCCESS................................")
                                exec_step = 6 #We are coming back here
                                jump = True

                        if not success:
                            rospy.logerr("....................FAIL................................")
                            self.exec_part.move_to_tray(tray_id)
                            self.part_plan.part.reset()
                            return False    

                    else:
                        rospy.logerr("....................FAIL................................")
                        self.exec_part.move_to_tray(tray_id)
                        self.part_plan.part.reset()
                        return False    
                else:                    
                    arm_actions.moveToolTipZY(0.3, incrementY, 1.4)
                    exec_step =+1 #STEP  - DONE

###################       STEP 6       ##########################################  
#STEP 6 - Check Falty Piece
#TODO Check falty if the part dropped at tray as well(improve performance)
            if(not jump and exec_step <= 6 and not self.exec_part.isInterupted()): 
                rospy.loginfo("\n\n[ExecuteBeltPart]: STEP 6 \n")
                rospy.sleep(1)#If we dont sleep we may not find the faulty one on this iteration
                if(self.part_plan.dest_tray_id == 1):
                    faulty_sensor_msg = global_vars.faulty_sensor1
                    sensor_name = "quality_control_sensor_1_frame"
                    angles_discard = STATIC_POSITIONS["disBelAgv1"]
                    solver = arm_actions.SolverType.AGV1
                elif(self.part_plan.dest_tray_id == 2): 
                    faulty_sensor_msg = global_vars.faulty_sensor1    
                    sensor_name = "quality_control_sensor_2_frame"
                    angles_discard = STATIC_POSITIONS["disBelAgv2"]
                    solver = arm_actions.SolverType.AGV2
                else:
                    rospy.logerr("\n\n\n[ExecutePart][BELT]: step7 failed. We do not know what to do yet")
                    rospy.logerr("[ExecutePart][BELT]: step6 failed. part_plan: {}\n\n\n".format(part_plan))
                    self.part_plan.part.reset()
                    return False

                falty = len(faulty_sensor_msg) > 0
                if falty:

                    rospy.loginfo("[ExecuteBeltPart][STEP6] - Falty part detected")

                    sensor_id, faulty_party_id = global_vars.tf_manager.find_part_name(
                        part_type, sensor_name)
                    time = rospy.get_time()
                    transforms_list = global_vars.tf_manager.get_transform_list(
                        faulty_party_id, 'world', time)
                    part_world_position, part_world_orientation = transform.transform_list_to_world(
                        transforms_list)

                    rospy.loginfo(
                        "[ExecuteBeltPart][STEP6] - Move Wait a bit above")
                    success = self.exec_part.move_wait_above_part(part_world_position=part_world_position,
                                                                  part_world_orientation=part_world_orientation,
                                                                  part_type=part_type, solver_type=solver,
                                                                  a_bit_above_value=0.015,
                                                                  time_to_execute_action=0.1)

                    rospy.loginfo("[ExecuteBeltPart][STEP6] - Go down untill get")
                    success = arm_actions.go_down_until_get_piece(world_position=part_world_position,
                                                                  world_orientation=part_world_orientation,
                                                                  part_type=part_type,
                                                                  time=1.5, ignore_height=False,
                                                                  distance=0.01, solver_type=solver)
                    

                    rospy.loginfo("[ExecuteBeltPart][STEP6] - Move ToolTip Up")  

                    success = self.exec_part.move_wait_above_part(part_world_position=part_world_position, 
                                                                part_world_orientation=part_world_orientation, 
                                                                part_type=part_type, solver_type=solver, 
                                                                a_bit_above_value=0.3, 
                                                                time_to_execute_action=0.3)
                    
                    
                    rospy.loginfo("[ExecuteBeltPart][STEP6] - Go to discard pos")  
                    arm_actions.set_arm_joint_values(list_of_joint_values=angles_discard,
                        time_to_execute_action=0.5)

                    arm_actions.check_arm_joint_values_published(list_of_joint_values=angles_discard)

                    rospy.loginfo("[ExecuteBeltPart][STEP6] - discard pos") 
                    success = gripper_actions.send_gripping_cmd_and_wait(False)
                    if(not success):
                        rospy.logerr("[ExecuteBeltPart]: step6 failed. We do not know what to do yet")
                        self.part_plan.part.reset()
                        return False

                    self.part_plan.part.reset()
                    return False
                else:
                    exec_step =+1 #STEP  - DONE

###################       STEP 7       ##########################################                
            if(exec_step <= 7 and not self.exec_part.isInterupted()): #STEP 7 - Move To TRAY
                rospy.loginfo("\n\n[ExecuteBeltPart]: STEP 7 \n")
                
                success = self.exec_part.move_to_tray(tray_id, force_check_piece=False, time=0.5)
                # success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecuteBeltPart]: step7 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP  - DONE

###################       STEP 8       ###################################
# STEP 8 - Check piece position and orientation
            if(not jump and exec_step <= 8 and not self.exec_part.isInterupted()):
                rospy.loginfo("\n\n[ExecutePart]: STEP 8 \n")

                part_position_at_tray, part_orientation_at_tray  = calculate_order_position(desired_part_pose, tray_id)
                
                camera_id, part_id = global_vars.tf_manager.find_part_name(part_name=part_name, sub_dad=camera_name)
                rospy.loginfo("[ExecBin]: DEBUG camera_name: {}; camera_id: {} ; part_id{}".format(camera_name, camera_id, part_id))
                if(len(camera_id) == 0 or len(part_id) == 0): #part not found
                    rospy.loginfo("\n\n\n[ExecBin]: step8 failed [part not found]. Reseting\n\n\n")
                    # arm_actions.moveToolTipZY(incrementZ=0.2, incrementY=incrementY, timeToGoal=0.2)
                    success = self.exec_part.move_to_tray(tray_id=tray_id, 
                                                force_check_piece=False, 
                                                force_grp_sts=False, 
                                                time=1)
                    # rospy.loginfo("[ExecutePart]: DEBUG SLEEL \n\n\n")
                    # rospy.sleep(10)
                    if not success:
                        #TODO MOVE UP A BIT
                        rospy.logerr("[ExecBin]: step8 failed. Could not get back to AGV")
                    # self.part_plan.part.reset()
                    return True #TODO CHECK all parts. not just the one
                # rospy.sleep(1)
                r = self.exec_part.find_part_any_agvs(part_id)#TODO any agv or a specific agv?
                
                
                part_world_position, part_world_orientation = r

                wrong_piece_position = checkPartOnTray(part_world_position, part_position_at_tray, "pos")
                wrong_piece_orientation = checkPartOnTray(part_world_orientation, part_orientation_at_tray, "ori")

                if (wrong_piece_position or wrong_piece_orientation):
                    #print ("_____________________________________________________________________________")
                    print ("\n\n\n\n " + str(part_world_position) + " \n\n\n\n\n")
                    print ("\n\n\n\n " + str(part_world_orientation) + " \n\n\n\n\n")
                    print ("\n\n\n\n " + str(part_type) + " \n\n\n\n\n") 
                    
                    # success = self.exec_part.move_wait_above_part(part_world_position, part_world_orientation, part_type, solver, 0.2)

                    rospy.loginfo(
                        "[ExecBin][STEP8] - Move Wait a bit above")
                    success = self.exec_part.move_wait_above_part(part_world_position=part_world_position,
                                                                  part_world_orientation=part_world_orientation,
                                                                  part_type=part_type, solver_type=solver,
                                                                  a_bit_above_value=0.1,
                                                                  time_to_execute_action=1,
                                                                  adjust=True)

                    if success:
                        rospy.loginfo(
                            "[ExecBin][STEP8] - go_down_until_get_piece")
                        # TODO DEBUG reduce time, but do test. it cannot be too
                        # fast
                        success = arm_actions.go_down_until_get_piece(world_position=part_world_position,
                                                                      world_orientation=part_world_orientation,
                                                                      part_type=part_type,
                                                                      time=3, ignore_height=False,
                                                                      distance=0.005, solver_type=solver,
                                                                      adjust=True)

                        arm_actions.moveToolTipZY(0.3, incrementY, 0.1)

                    rospy.logerr("........................................................................")
                    if success :
                        rospy.logerr("....................SUCCESS................................")
                        exec_step = 5 #We are coming back here
                        jump = True
                        continue

                    else:
                        rospy.logerr("[ExecBin]: step8 failed. We do not know what to do yet")
                        exec_step =+1 #STEP  - DONE

                done = True                

        return done


class ExecBin:

    def __init__(self, part_plan, exec_part):
        self.part_plan = part_plan
        self.exec_part = exec_part

    def execute(self):

        exec_step = 0
        failed_comple = False
        done = False
        STEP_4_MAX_ATTEMPT = 3
        step_4_attempt = 0

###################       STEP 0       ##########################################        

        while(not failed_comple and not done and not self.exec_part.isInterupted()):
            jump = False

            if(exec_step <= 0): #STEP 0 - Setup env
                part_origin = self.part_plan.pick_piece.origin.value
                part_type = self.part_plan.part.part_type
                part_world_position = self.part_plan.pick_piece.world_position
                part_world_orientation = self.part_plan.pick_piece.world_orientation
                tray_id = self.part_plan.dest_tray_id 
                desired_part_pose = self.part_plan.part.desired_pose
                exec_step =+1 #STEP 0 - DONE
                to_flip = self.part_plan.to_flip
                if to_flip:
                    rospy.loginfo("\n\n[ExecBin][ExecutePart]: STEP 0 : Part must be FLIPPED\n\n\n\n")
                gripper_actions.send_gripping_cmd_and_wait(False)

###################       STEP 1       ##########################################

            if(not jump and exec_step <= 1 and not self.exec_part.isInterupted()): #STEP 1 - Check Any BIN

                if part_origin == PickPlaces.ANY_BIN.value:
                    rospy.loginfo("\n\n[ExecBin]: STEP 1 \n")
                    # step 0 - get available pose for a part on any bin
                    camera_id, part_id = global_vars.tf_manager.find_part_name(part_type, sub_dad="logical_camera_bin")
                    # getting part id without the camera id
                    part_name = part_id[part_id.find(part_type):-6]
                    
                    if(len(camera_id) == 0 or len(part_id) == 0):
                        rospy.loginfo(
                            "[ExecBin]:Failed. No available part {} found".format(part_type))
                        self.part_plan.part.reset()
                        return False
                    r = self.exec_part.find_part_any_bin(camera_id, part_id, part_type)
                    if(r is None):
                        rospy.loginfo(
                            "[ExecBin]:Failed. No available part {} found".format(part_type))
                        self.part_plan.part.reset()
                        return False
                    part_world_position, part_world_orientation, time = r

                    exec_step =+1 #STEP 1 - DONE

###################       STEP 2       ##########################################  

            if(not jump and exec_step <= 2 and not self.exec_part.isInterupted()): #STEP 2 - Check Any BIN                  
                
                rospy.loginfo("\n\n[ExecBin][ExecutePart]: STEP 2 \n")
                # step 1 - move to position in front of the piece
                success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecBin][ExecutePart]: step2 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP 2 - DONE

###################       STEP 3       ##########################################                     

            if(not jump and exec_step <= 3 and not self.exec_part.isInterupted()): #STEP 3 - move to position a bit above the part

                rospy.loginfo("\n\n[ExecBin]: STEP 3 \n")
                max_attempt = 3
                attempt = 0
                success = False
                while(attempt < max_attempt and not success):            
                    success = self.exec_part.move_wait_above_part(part_world_position, 
                                                        part_world_orientation, 
                                                        part_type)
                    if not success: 
                        attempt +=1           
                        rospy.loginfo("[ExecBin]: step3 failed. attempt#" + str(attempt))

                if not success:                 
                    rospy.loginfo("[ExecBin] step3 failed completly" + str(attempt))
                    global_vars.tf_manager.add_part_id_to_bl(part_id)
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP 3 - DONE

###################       STEP 4       ########################################## 
#STEP 4 - go_down_until_get_piece           
            if(not jump and exec_step <= 4 and not self.exec_part.isInterupted()): 

                rospy.loginfo("\n\n[ExecBin]: STEP 4 \n")
                # 3 - go down until get the part

                success = arm_actions.go_down_until_get_piece(world_position=part_world_position, 
                                        world_orientation=part_world_orientation, 
                                        part_type=part_type, 
                                        time=1, ignore_height=False, 
                                        distance=0.02, solver_type=arm_actions.SolverType.BIN)
                if not success:
                    step_4_attempt += 1
                    rospy.logerr("[ExecBin]: step4 failed step_4_attempt={}".format(step_4_attempt))
                    
                    if(step_4_attempt > STEP_4_MAX_ATTEMPT):                    
                        #TODO insert at blacklist
                        rospy.logerr("\n\n[ExecBin]: step4 failed. add_part_id_to_bl part: {}\n\n".format(part_id))
                        global_vars.tf_manager.add_part_id_to_bl(part_id)
                        self.part_plan.part.reset()
                        return False
                    jump = True
                else:
                    exec_step =+1 #STEP 4 - DONE

###################       STEP 5       ##########################################

            if(not jump and exec_step <= 5 and not self.exec_part.isInterupted()): #STEP 5 - Going down to get part and flipping if needed

                rospy.loginfo("\n\n[ExecBin]: STEP 5 \n")

                # part must be flipped
                if to_flip:
                    success = self.exec_part.flip_part_bin(part_type)
                
                               
                success = self.exec_part.move_wait_front_part(part_world_position=part_world_position, 
                                    force_check_piece=True, force_grp_sts=True)
                

                if not success:
                    rospy.loginfo("[ExecBin]: step failed. Reseting")
                    # check what to do here
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP  - DONE

###################       STEP 6       ##########################################                
            if(not jump and exec_step <= 6 and not self.exec_part.isInterupted()): #STEP 6 - Move To TRAY
                rospy.loginfo("\n\n[ExecBin]: STEP 6 \n")
                success = self.exec_part.move_to_tray(tray_id)
                # success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecBin]: step6 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

            if(exec_step <= 6 and not self.exec_part.isInterupted()): #STEP 6 - Temporary Debug
                exec_step =+1 #STEP  - DONE

###################       STEP 7       ##########################################  
#STEP 7 - Put Part at tray              
            if(not jump and exec_step <= 7 and not self.exec_part.isInterupted()): 
                rospy.loginfo("\n\n[ExecBin]: STEP 7 \n")
                
                if(self.part_plan.dest_tray_id == 1):
                    rospy.logerr("\n\n\n[ExecBin]: step7 tray 1")                    
                    camera_name = AGVS_CAMERA["agv1"]
                    solver = arm_actions.SolverType.AGV1
                    incrementY=-0.1
                elif(self.part_plan.dest_tray_id == 2): 
                    rospy.logerr("\n\n\n[ExecBin]: step7 tray 2")                    
                    camera_name = AGVS_CAMERA["agv2"]
                    solver = arm_actions.SolverType.AGV2
                    incrementY=0.1
                rospy.loginfo("\n\n[ExecBin]: STEP 7 - deposit_at_tray \n")
                success = self.exec_part.deposit_at_tray(desired_part_pose=desired_part_pose, part_type=part_type, tray_id=tray_id, force_check_piece=True, time_to_execute_action=1, flipped_part = self.part_plan.to_flip)
                rospy.loginfo("\n\n[ExecBin]: STEP 7 - deposit_at_tray sucess: {} \n".format(success))
                #TODO Check if falty

                if not success:
                    rospy.loginfo("[ExecBin]: step7 failed. Reseting")
                    rospy.loginfo("[ExecBin][STEP7] - Move tool tip up")
                    arm_actions.moveToolTipZY(incrementZ=0.2, incrementY=incrementY, timeToGoal=0.3)

                    # waiting tf_manager update
                    rospy.sleep(2.5) #TODO changed from 5 to 2.5. Check if ok.
                    camera_id, part_id = global_vars.tf_manager.find_part_name(part_name=part_name, sub_dad=camera_name)
                    rospy.loginfo("[ExecBin]: DEBUG camera_id: {} ; part_id{}".format(camera_id, part_id))
                    if(len(camera_id) == 0 or len(part_id) == 0): #part not found
                        rospy.logerr("\n\n[ExecBin]: step7 failed [part not found]. Reseting\n\n")
                        # arm_actions.moveToolTipZY(incrementZ=0.2, incrementY=incrementY, timeToGoal=0.2)
                        success = self.exec_part.move_to_tray(tray_id=tray_id, 
                                                    force_check_piece=False, 
                                                    force_grp_sts=False, 
                                                    time=1)
                        # rospy.loginfo("[ExecutePart]: DEBUG SLEEL \n\n\n")
                        # rospy.sleep(10)
                        if not success:
                            #TODO MOVE UP A BIT
                            rospy.logerr("[ExecBin]: step7 failed. Could not get back to AGV")
                        self.part_plan.part.reset()
                        return False    
                    # rospy.sleep(1)

                    rospy.logerr("\n\n[ExecBin]: step7 failed [part dropped on tray].\n\n")

                    if(self.part_plan.dest_tray_id == 1):
                        faulty_sensor_msg = global_vars.faulty_sensor1
                    else:
                        faulty_sensor_msg = global_vars.faulty_sensor2

                    falty = len(faulty_sensor_msg) > 0
                    if not falty:
                        rospy.logerr("\n\n[ExecBin]: step7 Not faulty")

                        r = self.exec_part.find_part_any_agvs(part_id)#TODO any agv or a specific agv?
                        
                        if r is not None:
                            part_world_position, part_world_orientation = r

                            #print ("_____________________________________________________________________________")
                            print ("\n\n\n\n " + str(part_world_position) + " \n\n\n\n\n")
                            print ("\n\n\n\n " + str(part_world_orientation) + " \n\n\n\n\n")
                            print ("\n\n\n\n " + str(part_type) + " \n\n\n\n\n") 
                            
                            # success = self.exec_part.move_wait_above_part(part_world_position, part_world_orientation, part_type, solver, 0.2)

                            rospy.loginfo(
                                "[ExecBin][STEP7] - Move Wait a bit above")
                            success = self.exec_part.move_wait_above_part(part_world_position=part_world_position,
                                                                          part_world_orientation=part_world_orientation,
                                                                          part_type=part_type, solver_type=solver,
                                                                          a_bit_above_value=0.25,
                                                                          time_to_execute_action=1,
                                                                          adjust=True)



                            if success:
                                rospy.loginfo(
                                    "[ExecBin][STEP7] - go_down_until_get_piece")
                                # TODO DEBUG reduce time, but do test. it cannot be too
                                # fast
                                success = arm_actions.go_down_until_get_piece(world_position=part_world_position,
                                                                              world_orientation=part_world_orientation,
                                                                              part_type=part_type,
                                                                              time=3.5, ignore_height=False,
                                                                              distance=0.005, solver_type=solver,
                                                                              adjust=True)

                                arm_actions.moveToolTipZY(0.3, incrementY, 0.1)

                                rospy.logerr("........................................................................")
                                if success :
                                    rospy.logerr("....................SUCCESS................................")
                                    exec_step = 7 #We are coming back here
                                    jump = True

                            if not success:
                                rospy.logerr("\n\n\n\n[ExecBin]: step7 FAILED ")
                                rospy.logerr("[ExecBin]: ------------------------ ")
                                rospy.logerr("[ExecBin]: RESETING PART.PLAN() ")
                                self.exec_part.move_to_tray(tray_id=tray_id, force_check_piece=False, 
                                    force_grp_sts=False, time=0.5)
                                self.part_plan.part.reset()
                                return False    

                        else:
                            rospy.logerr("\n\n\n\n[ExecBin]: step7 FAILED ")
                            rospy.logerr("[ExecBin]: ------------------------ ")
                            rospy.logerr("[ExecBin]: RESETING PART.PLAN() ")
                            self.exec_part.move_to_tray(tray_id=tray_id, force_check_piece=False, 
                                    force_grp_sts=False, time=0.5)
                            self.part_plan.part.reset()
                            return False   

                    else:
                        rospy.loginfo("\n\n[ExecBin][ExecutePart]: STEP 7 - Part dropped and falty\n") 
                        exec_step =+1 #STEP  - DONE
                else:  
                    # rospy.loginfo("\n\n[ExecBin][ExecutePart]: STEP 7 - moveToolTipZY \n")                  
                    # arm_actions.moveToolTipZY(0.3, incrementY, 1.4)
                    rospy.loginfo("\n\n[ExecBin][ExecutePart]: STEP 7 - moveToolTipZY called\n") 
                    exec_step =+1 #STEP  - DONE

###################       STEP 8       ##########################################  
#STEP 8 - Check Falty Piece
#TODO Check falty if the part dropped at tray as well(improve performance)
            if(not jump and exec_step <= 8 and not self.exec_part.isInterupted()): 
                rospy.loginfo("\n\n[ExecBin]: STEP 8 \n")
                rospy.sleep(0.2)#If we dont sleep we may not find the faulty one on this iteration
                if(self.part_plan.dest_tray_id == 1):
                    faulty_sensor_msg = global_vars.faulty_sensor1
                    sensor_name = "quality_control_sensor_1_frame"
                    angles_discard = STATIC_POSITIONS["disBelAgv1"]
                    angles_discard_open = STATIC_POSITIONS["disBelAgv1Open"]
                    angles_discard_back = STATIC_POSITIONS["disBelAgv1Back"]
                    solver = arm_actions.SolverType.AGV1
                elif(self.part_plan.dest_tray_id == 2): 
                    faulty_sensor_msg = global_vars.faulty_sensor2    
                    sensor_name = "quality_control_sensor_2_frame"
                    angles_discard = STATIC_POSITIONS["disBelAgv2"]
                    angles_discard_open = STATIC_POSITIONS["disBelAgv2Open"]
                    angles_discard_back = STATIC_POSITIONS["disBelAgv2Back"]
                    solver = arm_actions.SolverType.AGV2
                else:
                    rospy.logerr("\n\n\n[ExecutePart]: step8 failed. We do not know what to do yet")
                    rospy.logerr("[ExecutePart]: step8 failed. part_plan: {}\n\n\n".format(part_plan))
                    self.part_plan.part.reset()
                    return False

                falty = len(faulty_sensor_msg) > 0
                if falty:

                    rospy.loginfo("[ExecBin][STEP8] - Falty part detected")

                    sensor_id, faulty_party_id = global_vars.tf_manager.find_part_name(
                        part_type, sensor_name)
                    time = rospy.get_time()
                    transforms_list = global_vars.tf_manager.get_transform_list(
                        faulty_party_id, 'world', time)
                    part_world_position, part_world_orientation = transform.transform_list_to_world(
                        transforms_list)

                    rospy.loginfo(
                        "[ExecBin][STEP8] - Move Wait a bit above")
                    success = self.exec_part.move_wait_above_part(part_world_position=part_world_position,
                                                                  part_world_orientation=part_world_orientation,
                                                                  part_type=part_type, solver_type=solver,
                                                                  a_bit_above_value=0.025,
                                                                  time_to_execute_action=0.1)

                    rospy.loginfo("[ExecBin][STEP8] - Go down untill get")
                    success = arm_actions.go_down_until_get_piece(world_position=part_world_position,
                                                                  world_orientation=part_world_orientation,
                                                                  part_type=part_type,
                                                                  time=3.5, ignore_height=False,
                                                                  distance=0.01, solver_type=solver)
                    # rospy.sleep(5)  # TODO REMOVE

                    rospy.loginfo("[ExecBin][STEP8] - Move ToolTip Up")

                    success = self.exec_part.move_wait_above_part(part_world_position=part_world_position,
                                                                  part_world_orientation=part_world_orientation,
                                                                  part_type=part_type, solver_type=solver,
                                                                  a_bit_above_value=0.3,
                                                                  time_to_execute_action=0.3)

                    
                    self.exec_part.move_to_tray(tray_id, time=1)

                    if(self.part_plan.dest_tray_id == 1):
                        rospy.loginfo("[ExecBin][STEP8] - Go to discard pos")
                        arm_actions.set_arm_joint_values(list_of_joint_values=angles_discard,
                            time_to_execute_action=0.5)
                        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles_discard)



                    

                    rospy.loginfo("[ExecBin][STEP8] - Open elbow")


                    rospy.loginfo("[ExecBin][STEP8] - Go to discard pos")
                    arm_actions.set_arm_joint_values(list_of_joint_values=angles_discard_open,
                        time_to_execute_action=0.5)

                    arm_actions.check_arm_joint_values_published(list_of_joint_values=angles_discard_open)


                    rospy.loginfo("[ExecBin][STEP8] - discard pos") 
                    success = gripper_actions.send_gripping_cmd_and_wait(False)
                    rospy.loginfo("[ExecBin][STEP8]  send_gripping_cmd_and_wait success: {}".format(success)) 

                    
                    arm_actions.set_arm_joint_values(list_of_joint_values=angles_discard_back,
                        time_to_execute_action=0.5)

                    arm_actions.check_arm_joint_values_published(list_of_joint_values=angles_discard_back)


                    if(not success):
                        rospy.logerr("\n\n\n[ExecBin]: step8 failed. We do not know what to do yet")
                        rospy.logerr("\n\n\n[ExecBin]: send_gripping_cmd_and_wait(False) Failed\n\n\n")
                        
                        self.part_plan.part.reset()
                        return False

                    self.part_plan.part.reset()
                    return False
                else:
                    exec_step =+1 #STEP  - DONE
                


###################       STEP 9       ###################################
# STEP 9 - Move To TRAY
            if(not jump and exec_step <= 9 and not self.exec_part.isInterupted()):
                rospy.loginfo("\n\n[ExecutePart]: STEP 9 \n")

                success = self.exec_part.move_to_tray(
                    tray_id, force_check_piece=False, time=0.5)
                # success = self.exec_part.move_wait_front_part(part_world_position)
                if not success:
                    rospy.loginfo("[ExecutePart]: step9 failed. Reseting")
                    self.part_plan.part.reset()
                    return False

                exec_step =+1 #STEP  - DONE


###################       STEP 10       ###################################
# STEP 10 - Check piece position and orientation
            if(not jump and exec_step <= 10 and not self.exec_part.isInterupted()):
                rospy.loginfo("\n\n[ExecutePart]: STEP 10 \n")

                part_position_at_tray, part_orientation_at_tray  = calculate_order_position(desired_part_pose, tray_id)
                
                camera_id, part_id = global_vars.tf_manager.find_part_name(part_name=part_name, sub_dad=camera_name)
                rospy.loginfo("[ExecBin]: DEBUG camera_name:{}; camera_id: {} ; part_id{}".format(camera_name, camera_id, part_id))
                if(len(camera_id) == 0 or len(part_id) == 0): #part not found
                    rospy.loginfo("[ExecBin]: step10 failed [part not found]. Reseting")
                    # arm_actions.moveToolTipZY(incrementZ=0.2, incrementY=incrementY, timeToGoal=0.2)
                    success = self.exec_part.move_to_tray(tray_id=tray_id, 
                                                force_check_piece=False, 
                                                force_grp_sts=False, 
                                                time=1)
                    # rospy.loginfo("[ExecutePart]: DEBUG SLEEL \n\n\n")
                    # rospy.sleep(10)
                    if not success:
                        #TODO MOVE UP A BIT
                        rospy.logerr("[ExecBin]: step10 failed. Could not get back to AGV")
                    self.part_plan.part.reset()
                    return False
                # rospy.sleep(1)
                r = self.exec_part.find_part_any_agvs(part_id)#TODO any agv or a specific agv?
                
                
                part_world_position, part_world_orientation = r

                wrong_piece_position = checkPartOnTray(part_world_position, part_position_at_tray, "pos")
                wrong_piece_orientation = checkPartOnTray(part_world_orientation, part_orientation_at_tray, "ori")

                if (wrong_piece_position or wrong_piece_orientation):
                    #print ("_____________________________________________________________________________")
                    print ("\n\n\n\n " + str(part_world_position) + " \n\n\n\n\n")
                    print ("\n\n\n\n " + str(part_world_orientation) + " \n\n\n\n\n")
                    print ("\n\n\n\n " + str(part_type) + " \n\n\n\n\n") 
                    
                    # success = self.exec_part.move_wait_above_part(part_world_position, part_world_orientation, part_type, solver, 0.2)

                    rospy.loginfo(
                        "[ExecBin][STEP10] - Move Wait a bit above")
                    success = self.exec_part.move_wait_above_part(part_world_position=part_world_position,
                                                                  part_world_orientation=part_world_orientation,
                                                                  part_type=part_type, solver_type=solver,
                                                                  a_bit_above_value=0.25,
                                                                  time_to_execute_action=1,
                                                                  adjust=True)

                    if success:
                        rospy.loginfo(
                            "[ExecBin][STEP10] - go_down_until_get_piece")
                        # TODO DEBUG reduce time, but do test. it cannot be too
                        # fast
                        success = arm_actions.go_down_until_get_piece(world_position=part_world_position,
                                                                      world_orientation=part_world_orientation,
                                                                      part_type=part_type,
                                                                      time=3, ignore_height=False,
                                                                      distance=0.005, solver_type=solver,
                                                                      adjust=True)

                        arm_actions.moveToolTipZY(0.3, incrementY, 0.1)

                    rospy.logerr("........................................................................")
                    if success :
                        rospy.logerr("....................SUCCESS................................")
                        exec_step = 7 #We are coming back here
                        jump = True
                        continue

                    else:
                        rospy.logerr("[ExecBin]: step10 failed. We do not know what to do yet")
                        exec_step =+1 #STEP  - DONE

                done = True

###################       STEP 11       ##################################
            # if(exec_step <= 11 and not self.exec_part.isInterupted()): #STEP
            # 11 - Temporary Debug

            #     rospy.loginfo("\n\n[ExecutePart]: STEP 11 \n")
            #     gripper_actions.send_gripping_cmd(toGrip=False)
            #     gripper_actions.wait_for_gripper(toGrip=False, max_wait=5, inc_sleep=0.01)
            #     rospy.sleep(0.5)

            #     done = True

        return done


class ExecutePart:

    def __init__(self, partPlan):
        self.partPlan = partPlan
        self.interrupt = False

    def interupt_call_back():
        self.interrupt = True
        
    def isInterupted(self):
        return self.interrupt    

    def check_gripper(self, toGrp):
        return global_vars.gripper_state.attached == toGrp

    def move_wait_belt(self, part_world_position, 
                        force_check_piece=False, force_grp_sts=True):

        rospy.loginfo("[ExecutePart]: move_wait_front_part: "+ str(part_world_position))
        if(force_check_piece):
            grpOK = self.check_gripper(force_grp_sts)
            if (not grpOK):
                rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                return False

        angles_start = arm_actions.go_to_belt_start()
        # checking joint states
        success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles_start,
                                            accError=[0.009, 0.009, 0.009, 0.009,
                                               0.015, 0.015, 0.009, 0.009, 0.1],
                                            force_check_piece=False, force_grp_sts=True)
        if not success:
            rospy.logerr("[ExecutePart]: move_wait_belt failed")
            return False

        rospy.loginfo("[ExecutePart]:move_wait_belt moved to angles_start")

        angles = arm_actions.go_to_belt()

        # checking joint states
        success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles,
                                            accError=[0.009, 0.009, 0.009, 0.009,
                                               0.015, 0.015, 0.009, 0.009, 0.1],
                                            force_check_piece=False, force_grp_sts=True)
        if not success:
            rospy.logerr("[ExecutePart]: move_wait_front_part failed")
            return False
        else:
            rospy.loginfo("[ExecutePart]:move_wait_belt moved to angles_rest")
            if(force_check_piece):
                grpOK = self.check_gripper(force_grp_sts)
                if (not grpOK):
                    rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                    return False
                else:
                    return True
            else:
                return True

    def move_towards_piece_on_belt(self, part_world_position, part_world_orientation,
                                    part_world_tf_time, part_type):

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

        arm_actions.set_arm_joint_values(pos_robot, time_diff)

        back_position = pos_robot[2]
        while not global_vars.gripper_state.attached:
            gripper_actions.send_gripping_cmd(toGrip=True)
            pos_robot[1] -= 0.2 

            if pos_robot[2] < list_joint_values[2]:
                pos_robot[2] += 0.08

            if pos_robot[2] > list_joint_values[2]:
                pos_robot[2] = list_joint_values[2]

            arm_actions.set_arm_joint_values(pos_robot, 1)

            if rospy.get_time()-timer >= 20 or pos_robot[1] <= -2.1:
                return False

            rospy.sleep(1)


        # raising arm again
        pos_robot[0] = STATIC_POSITIONS["rest_position"][0]
        pos_robot[2] = back_position
        arm_actions.set_arm_joint_values(pos_robot, 0.5)
        rospy.sleep(0.5)#TODO can we change to check_angles?

        return True

    def move_wait_front_part(self, part_world_position, 
                        force_check_piece=False, force_grp_sts=True):
        rospy.loginfo("[ExecutePart]: move_wait_front_part: "+ str(part_world_position))
        if(force_check_piece):
            grpOK = self.check_gripper(force_grp_sts)
            if (not grpOK):
                rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                return False


        angles = arm_actions.go_to_part_bin_front(part_world_position)

        # checking joint states
        success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)
        if not success:
            rospy.logerr("\n[ExecutePart]: move_wait_front_part failed")
            rospy.logerr("[ExecutePart]: part_world_position: " + str(part_world_position))
            rospy.logerr("[ExecutePart]: force_check_piece: " + str(force_check_piece))
            rospy.logerr("[ExecutePart]: force_grp_sts: " + str(force_grp_sts))
            rospy.logerr("[ExecutePart]: list_of_joint_values: " + str(angles))
            rospy.logerr("\n\n")
            return False
        else:
            if(force_check_piece):
                grpOK = self.check_gripper(force_grp_sts)
                if (not grpOK):
                    rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                    return False
                else:
                    return True
            else:
                return True

    def move_wait_above_part(self, part_world_position, part_world_orientation, part_type, 
        solver_type=arm_actions.SolverType.BIN, 
        a_bit_above_value=0.025, time_to_execute_action=1, 
        accError=[0.009, 0.009, 0.009, 0.009,0.015, 0.015, 0.009, 0.009, 0.009], adjust=False):
        rospy.loginfo("[ExecutePart]: move_wait_above_part: "+ str(part_world_position))
        list_joint_values = arm_actions.go_to_position_a_bit_above_part(
            world_position=part_world_position,
            world_orientation=part_world_orientation,
            part_type=part_type, 
            time_to_execute_action=time_to_execute_action, 
            solver_type=solver_type,
            a_bit_above_value=a_bit_above_value,
            adjust=adjust)

        success = arm_actions.check_arm_joint_values_published(
            list_of_joint_values=list_joint_values,
            accError=accError)       
        if not success:
            rospy.logerr("[ExecutePart]: move_wait_above_part failed")
        return success

    def find_part_any_bin(self, camera_id, part_id, part_type):
        if len(camera_id) > 0:
            rospy.loginfo("[ExecutePart][find_part_any_bin]: part: "+ str(part_id))

            time = rospy.get_time()
            # getting position and orientation from the part
            transforms_list = global_vars.tf_manager.get_transform_list(part_id, 'world', None)#DEBUG
            # rospy.logerr("[find_part_any_bin]:" + str(transforms_list))
            if(transforms_list is not None and len(transforms_list) > 0):
                time = transforms_list[0]['secs']
                t, a = transform.transform_list_to_world(transforms_list)
                return t, a, time

    def find_part_any_belt(self, camera_id, part_id, part_type):
        if len(camera_id) > 0:
            rospy.loginfo("[ExecutePart][find_part_any_bin]: part: "+ str(part_id))


            # getting position and orientation from the part
            transforms_list = global_vars.tf_manager.get_transform_list(part_id, 'world')#DEBUG
            # rospy.logerr("[find_part_any_bin]:" + str(transforms_list))
            if(transforms_list is not None and len(transforms_list) > 0):
                time = transforms_list[0]['time']
                t, a = transform.transform_list_to_world(transforms_list)
                return t, a, time
            

    def find_part_any_agvs(self, part_id):
        rospy.loginfo("[ExecutePart][find_part_any_agvs]: part: "+ str(part_id))
        # getting bin id from the part
        time = rospy.get_time()
        # getting position and orientation from the part
        transforms_list = global_vars.tf_manager.get_transform_list(part_id, 'world', time)
        # print "\n\n-----------------------------------------"
        # print transforms_list
        # print "\n\n-----------------------------------------"
        if len(transforms_list) > 0:
            return transform.transform_list_to_world(transforms_list)
        

        

    def move_to_tray(self, tray_id, force_check_piece=True, force_grp_sts=True, time=2):
        tray_name = "agv" + str(tray_id)
        rospy.loginfo("[ExecutePart]: move_to_tray: "+ tray_name)

        if(force_check_piece):
            grpOK = self.check_gripper(force_grp_sts)
            if (not grpOK):
                rospy.logerr("[ExecutePart]:move_wait_front_part - Gripper failed!")
                return False

        angles = arm_actions.go_to_tray_position(tray_name)
        success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)


        if not success:
            rospy.logerr("[ExecutePart]: move_to_tray failed")
            return False
        else:
            if(force_check_piece):
                grpOK = self.check_gripper(force_grp_sts)
                if (not grpOK):
                    rospy.logerr("[ExecutePart]:move_to_tray - Gripper failed!")
                    return False
                else:
                    return True
            else:
                return True

    def deposit_at_tray(self, desired_part_pose, part_type, tray_id, force_check_piece=False, force_grp_sts=True, time_to_execute_action=1, accError=[0.009, 0.009, 0.009, 0.009,0.015, 0.015, 0.01, 0.009, 0.009], flipped_part=False):
        # calculate position of the part at tray
        solver_type = arm_actions.SolverType.AGV1 if tray_id == 1 else arm_actions.SolverType.AGV2
        part_position_at_tray, part_orientation_at_tray  = calculate_order_position(desired_part_pose, tray_id)


        if(force_check_piece):
            grpOK = self.check_gripper(force_grp_sts)
            if (not grpOK):
                rospy.logerr("[ExecutePart]:deposit_at_tray - Gripper failed!")
                return False

        # move a bit above the position
        max_attempt = 3
        attempt = 0
        success = False
        while(attempt < max_attempt and not success):            
            success = self.move_wait_above_part(part_position_at_tray, 
                                                part_orientation_at_tray, 
                                                part_type,
                                                solver_type=solver_type,
                                                time_to_execute_action=time_to_execute_action,                                                
                                                accError=accError)
            if not success: 
                attempt +=1           
                rospy.loginfo("[ExecutePart]: failed. attempt#" + str(attempt))
            else:
                if("pulley" in part_type):
                    rospy.loginfo("[ExecutePart]: deposit_at_tray pullet o:" + str(part_orientation_at_tray))
                    a_bit_above_value = 0
                    #flipped_part = self.part_plan.to_flip
                    #if(part_orientation_at_tray[0]!= 0 or part_orientation_at_tray[1] != 0): #TODO check not desired but how it is now
                    if(flipped_part): 
                        a_bit_above_value = -0.01
                    rospy.sleep(0.5)
                    success = self.move_wait_above_part(part_position_at_tray, 
                                        part_orientation_at_tray, 
                                        part_type,
                                        solver_type=solver_type,
                                        time_to_execute_action=0.5,
                                        a_bit_above_value=a_bit_above_value,
                                        accError=accError)
                    if not success: 
                        attempt +=1           
                        rospy.loginfo("[ExecutePart]: failed. attempt#" + str(attempt))

        if not success:
            rospy.logerr("[ExecutePart]: deposit_at_tray failed")
            return False
        else:
            if(force_check_piece):
                
                grpOK = self.check_gripper(force_grp_sts)
                if (not grpOK):
                    rospy.logerr("[ExecutePart]:deposit_at_tray - Gripper failed to complete the movement holding part!")
                    return False
                else:
                    rospy.sleep(0.1) # waiting a little bit to deposit part
                    success = gripper_actions.send_gripping_cmd(toGrip=False)

                    if success:
                        return True
                    else:
                        rospy.logerr("[ExecutePart]:deposit_at_tray - Gripper failed to drop part!")
                        return False
            else:
                return True


    def flip_part_bin(self, part_type):

        rospy.logerr("[flip_part_bin]: moveToolTip(0.3, 0.1, 1.4)")

        arm_actions.moveToolTip(0.2, 0.1, 1.4)

        rospy.logerr("[flip_part_bin]: turnWrist(0.01)")
        
        arm_actions.turnWrist(0.01)

        rospy.logerr("[flip_part_bin]: MoveSideWays(0.022)")
        
        arm_actions.MoveSideWays(0.022)

        accError=[0.009, 0.009, 0.009, 0.009,
                                               0.1, 0.1, 0.009, 0.009, 0.009]

        rospy.logerr("[flip_part_bin]: moveToolTip(-0.17, 0.13, 1.0, accError=accError)")
        
        arm_actions.moveToolTip(-0.17, 0.13, 1.0, accError=accError)

        rospy.sleep(0.1)

        rospy.logerr("[flip_part_bin]: send_gripping_cmd(toGrip=False)")

        gripper_actions.send_gripping_cmd(toGrip=False)

        rospy.logerr("[flip_part_bin]: MoveSideWays(0.02)")
        
        arm_actions.MoveSideWays(0.02)

        rospy.logerr("[flip_part_bin]: wait_for_gripper(toGrip=False, max_wait=2, inc_sleep=0.01)")
        
        gripper_actions.wait_for_gripper(toGrip=False, max_wait=2, inc_sleep=0.01)

        
        rospy.logerr("[flip_part_bin]: turnWrist(-1.5707963268)")        
        
        arm_actions.turnWrist(-1.5707963268)

        rospy.logerr("[flip_part_bin]: moveToolTip(0.4, 0, 0.3)") 
        # Move ToolTip UP
        arm_actions.moveToolTip(0.4, 0, 0.3)




        rospy.logerr("[flip_part_bin]: MoveSideWays(-0.4)")  
        #Move a bit to the other side
        arm_actions.MoveSideWays(-0.4)
        
        rospy.sleep(0.8)
        
        rospy.logerr("[flip_part_bin]: moveToolTip(-0.01, 0.23, 0.2)")
        # Move ToolTip Down
        arm_actions.moveToolTip(-0.01, 0.23, 0.2)
        
        rospy.sleep(0.7)
        
        rospy.logerr("[flip_part_bin]: MoveSideWays(0.3)")
         # Move a bit to the other side
        arm_actions.MoveSideWays(0.3)
        
        rospy.sleep(0.7)

        camera_id, part_id = global_vars.tf_manager.find_part_name(part_type)
        if(camera_id is None or part_id is None):
            rospy.loginfo(
                "[ExecutePart]:Failed. No available part {} found".format(part_type))
            self.part_plan.part.reset()
            return False
        r = self.find_part_any_bin(camera_id, part_id, part_type)
        if(r is None):
            rospy.loginfo(
                "[ExecutePart]:Failed. No available part {} found".format(part_type))
            self.part_plan.part.reset()
            return False
        part_world_position, part_world_orientation, time = r

        rospy.sleep(1)

        self.move_wait_above_part(part_world_position,
                                  part_world_orientation,
                                  part_type)

        success = arm_actions.go_down_until_get_piece(part_world_position,
                                                      part_world_orientation,
                                                      part_type, ignore_height=True)

        # Move ToolTip UP
        self.move_wait_above_part(part_world_position,
                                  part_world_orientation,
                                  part_type, a_bit_above_value=0.3)

        return success

    def turn_pulley_yellow_bar(self, part_world_orientation):
        rospy.loginfo("[turn_pulley_yellow_bar] Turning pulley at yellow bar")
        

        rospy.loginfo("[turn_pulley_yellow_bar] Move to bin8")
        angles = STATIC_POSITIONS["bin8"]
        arm_actions.set_arm_joint_values(angles, 3)

        rospy.loginfo("[turn_pulley_yellow_bar] Move Up 1")
        angles = [2.62, 1.15, -1.80, 3.14, 3.34, -1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)

        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=True, force_grp_sts=True)

        rospy.loginfo("[turn_pulley_yellow_bar] Move Up 2")
        angles = [2.62, 1.15, -2.18, 3.14, 3.34, 1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                                    force_check_piece=True, force_grp_sts=True)
        

        angles = [1.61, 1.15, -2.18, 3.14, 5.03, 1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=True, force_grp_sts=True)

        angles = [1.44, 1.15, -1.79, 3.14, 5.03, 1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Place 1")
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=True, force_grp_sts=True)

        
        gripper_actions.send_gripping_cmd(toGrip=False)


        angles = [1.50, 1.15, -1.81, 3.14, 5.03, 1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Out 1")
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)

        angles = [1.75, 1.15, -2.19, 3.14, 5.03, 1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Out 2")
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)

        angles = [1.00, 1.15, -1.90, 3.14, 5.03, -1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Out 3")
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)

        angles = [1.00, 1.15, -1.90, 3.14, 5.03, -1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Adjust 1")
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)

        angles = [0.50, 1.15, -1.50, 3.14, 5.60, -1.57, 0]
        arm_actions.set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Adjust 2")
        arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                            force_check_piece=False, force_grp_sts=True)

        #angles = [0.10, 1.18, -1.19, 3.14, 5.80, -1.57, 0]
        #self.moveTo(angles, 4)
        #teste = ActionSleep(3, "Take")
        # teste.execute_action()

        gripper_actions.send_gripping_cmd(toGrip=True)
        angles = [-0.13, 1.18, -1.08, 3.14, 5.92, -1.57, 0]
        arm_actions.set_arm_joint_values(angles, 4)
        rospy.loginfo("[turn_pulley_yellow_bar] Take Test")

        gripper_actions.wait_for_gripper(toGrip=True, max_wait=5, inc_sleep=0.005)
        
        angles = [0.11, 1.18, -1.24, 3.14, 5.68, -1.57, 1.57 - part_world_orientation[2]]
        arm_actions.set_arm_joint_values(angles, 1)
        rospy.loginfo("[turn_pulley_yellow_bar] Out with Pulley")

        success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                        force_check_piece=True, force_grp_sts=True)
        if success:

            angles = [1.49, 1.18, -2.25, 3.14, 5.18, -1.57, 1.57 - part_world_orientation[2]]
            arm_actions.set_arm_joint_values(angles, 1)
            rospy.loginfo("[turn_pulley_yellow_bar] Adjust 1")
            
            success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                force_check_piece=True, force_grp_sts=True)


            angles = [2.12, 1.18, -2.12, 3.14, 4.30, -1.57, 1.57 - part_world_orientation[2]]
            arm_actions.set_arm_joint_values(angles, 1)
            rospy.loginfo("[turn_pulley_yellow_bar] Adjust 2")

            success = arm_actions.check_arm_joint_values_published(list_of_joint_values=angles, 
                                force_check_piece=True, force_grp_sts=True)

            return success

        return False


    def execute_belt(self, part_origin):

        exec_belt = ExecBelt(part_plan=self.partPlan, exec_part=self)
        success = exec_belt.execute()

        return success

    def execute_bin(self, part_origin):

        exec_bin = ExecBin(part_plan=self.partPlan, exec_part=self)
        success = exec_bin.execute()

        return success

    def execute(self):
        # check where the part is, bin or belt
        part_origin = self.partPlan.pick_piece.origin.value
        self.partPlan.part.set_time_started_if_not_already(rospy.Time.now())
        success = False
        # if part is on the bin:
        if "bin" in part_origin:
            rospy.loginfo("\n\n\n[Execution] START-BIN partPlan: {}\n\n\n".format(self.partPlan))
            success = self.execute_bin(part_origin)

        # if part is on the belt
        elif "belt" in part_origin:
            rospy.loginfo("\n\n\n[Execution] START-BELT partPlan: {}\n\n\n".format(self.partPlan))
            success = self.execute_belt(part_origin)

                # if part is on the tray
        elif "fail" in part_origin:
            rospy.loginfo("\n\n\n[Execution] START-FAILF partPlan: {}\n\n\n".format(self.partPlan))
            success = self.partPlan.part.set_done()

        # if part is on the tray

        elif "tray" in part_origin:
            rospy.loginfo("\n\n\n[Execution] START-TRAY partPlan: {}\n\n\n".format(self.partPlan))
            success = False

        return success


def send_agv(kit, tray_id):
    agvServiceName = "/ariac/agv{0}".format(tray_id)
    rospy.loginfo("sending 'kit_type: " + kit.kit_type +
                  " to : " + agvServiceName)
    rospy.wait_for_service(agvServiceName)
    try:
        send_agv = rospy.ServiceProxy(
            agvServiceName, AGVControl)
        success = send_agv(kit.kit_type)
        rospy.sleep(0.1)
        return success
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to notify agv %s: %s" % (self.kit_type, exc))
        return False
