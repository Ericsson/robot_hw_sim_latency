#!/usr/bin/env python
import rospy
import order_utils
import execution
import global_vars
import math
import sys

from constants import *
from utils import PickPlaces
from osrf_gear.srv import GetMaterialLocations


class PickPiece:

    def __init__(self, origin, world_position, world_orientation):
        self.origin             = origin #binX, belt or agvN
        self.world_position     = world_position
        self.world_orientation  = world_orientation

    def __str__(self):

        return 'PickPiece origin:{}; world_position:{}; world_orientation:{}'.format(
            self.origin, self.world_position, self.world_orientation)

    def __repr__(self):
        return 'PickPiece origin:{}; world_position:{}; world_orientation:{}'.format(
            self.origin, self.world_position, self.world_orientation)

class KitPlan:

    def __init__(self, kit, working_agv, list_part_plan):
        self.kit = kit
        self.working_agv = working_agv
        self.dest_tray_id = working_agv.agv_id
        self.list_part_plan = list_part_plan
        self.planning_score = -1

    def compute_score(self):
        score = 0
        for partPlan in list_part_plan:
            score += partPlan.compute_score()
        return score  

    def __str__(self):

        return 'KitPlan kit:{}; working_agv:{}; planning_score:{}'.format(
            self.kit, self.working_agv, self.planning_score)

    def __repr__(self):
        return 'KitPlan kit:{}; working_agv:{}; planning_score:{}'.format(
            self.kit, self.working_agv, self.planning_score)

class PartPlan:

    def __init__(self, part, pick_piece, kit_plan):
        self.part = part
        self.kit_plan = kit_plan
        self.dest_tray_id = kit_plan.dest_tray_id
        self.pick_piece = pick_piece
        self.planning_score = -1
        # in quaternion representation 1 = PI
        if (self.part.desired_pose.orientation.x != 0 or self.part.desired_pose.orientation.y != 0) and self.part.part_type == "pulley_part":
            self.to_flip = True
        else:
            self.to_flip = False

    def compute_score(self):
        pass

    def __str__(self):

        return 'PartPlan part:{}; kit_plan:{}; pick_piece:{}; planning_score:{}'.format(
            self.part, self.kit_plan, self.pick_piece, 
            self.planning_score)

    def __repr__(self):
        return 'PartPlan part:{}; kit_plan:{}; pick_piece:{}; planning_score:{}'.format(
            self.part, self.kit_plan, self.pick_piece, 
            self.planning_score)




class WorkingAgv:

    def __init__(self, agv_id):
        self.agv_id = agv_id
        self.reserved = False
        self.reserved_for_kit = None

    def __str__(self):
        return 'WorkingAgv agv_id:{}; reserved:{}; reserved_for_kit_id:{}'.format(
            self.agv_id, self.reserved, self.reserved_for_kit)

    def __repr__(self):
        return 'WorkingAgv agv_id:{}; reserved:{}; reserved_for_kit_id:{}'.format(
            self.agv_id, self.reserved, self.reserved_for_kit.kit_id)

    def reserve(self, kit):
        self.reserved = True
        self.reserved_for_kit = kit
    
    def release(self):
        self.reserved = False
        self.reserved_for_kit = None

    def is_available(self):
        available = self.is_ready_to_deliver() and not self.reserved
        rospy.logdebug("[WorkingAgv] is_available: " + str(available))
        return available

    def is_ready_to_deliver(self):        
        if(self.agv_id == 1):
            status = global_vars.agv1_status
        elif(self.agv_id == 2): 
            status = global_vars.agv2_status
        else:
            rospy.logerr("[WorkingAgv] We really should not be here")

        rospy.logdebug("[WorkingAgv] is_ready_to_deliver: " + str(status))
        #possible status: [ready_to_deliver, preparing_to_deliver, delivering, returning]
        if(status != "ready_to_deliver"):
            return False

        return True



class Scheduler:



    def __init__(self, competition):
        self.order_list = []
        self.finished = False
        self.competition = competition
        self.configure_agvs()
        self.start_time = None

    #[TODO] Are we sure to have always two?
    def configure_agvs(self):

        agv1 = WorkingAgv(1)    
        agv2 = WorkingAgv(2)
        self.agvs = [agv1, agv2]


    def append_order(self, order):
        self.order_list.append(order)

    def isFinished(self):
        return self.finished

    def setFinished(self, status):
        self.finished = status

    # def check_time_out_and_complete(self):
    #     rospy.loginfo("[Scheduler] Check Timeout - #Ordes: {}".format(len(self.order_list)))
    #     for order in self.order_list:
    #         for kit in order.kits:
    #             time_now = rospy.Time.now()
    #             rospy.loginfo("[Scheduler] Check Timeout - Kit: {}; Now: {}; Started:{}".format(kit, time_now, kit.time_started))
    #             if(kit.time_started is not None):
    #                 elapsed = time_now - kit.time_started
    #                 rospy.loginfo("[Scheduler] Check Timeout - Kit: {}; Elapsed: {}; TO_TIMETOUT(s):{}".format(kit, elapsed, (KIT_TIMEOUT-elapsed).to_sec()))
    #                 if elapsed > KIT_TIMEOUT:
    #                     kit.state = order_utils.Status.DONE
    #                     rospy.logerr("\n\n\n[Scheduler] Kit completed with issues")
    #                     rospy.loginfo("[Scheduler] Sending AGV")
    #                     success_agv_cmd = execution.send_agv(kit, kit.plan.dest_tray_id)
    #                     if(success_agv_cmd):
    #                         kit.plan.working_agv.release()
    #                     else:
    #                         rospy.loginfo("[Scheduler] Not Comp Implemented Yet - Could not send Agv Cmd")  
    #                         attempt_max = 3
    #                         attempt = 0
    #                         while  attempt <  attempt_max and not success_agv_cmd:
    #                             success_agv_cmd = execution.send_agv(kit, kit.plan.dest_tray_id)
    #                         if(success_agv_cmd):
    #                             kit.kit_plan.working_agv.release()      

    def check_time_out_and_complete(self):
        rospy.loginfo("[Scheduler] Check Timeout - #Ordes: {}".format(len(self.order_list)))
        time_now = rospy.Time.now()
        elapsed = time_now - self.start_time
        if elapsed > COMP_TIMEOUT:     

            for order in self.order_list:
                for kit in order.kits:                    
                    if(kit.time_started is not None):
                        kit.state = order_utils.Status.DONE
                        rospy.logerr("\n\n\n[Scheduler] Kit completed with issues")
                        rospy.loginfo("[Scheduler] Sending AGV")
                        success_agv_cmd = execution.send_agv(kit, kit.plan.dest_tray_id)
                        if(success_agv_cmd):
                            kit.plan.working_agv.release()
                        else:
                            rospy.loginfo("[Scheduler] Not Comp Implemented Yet - Could not send Agv Cmd")  
                            attempt_max = 3
                            attempt = 0
                            while  attempt <  attempt_max and not success_agv_cmd:
                                success_agv_cmd = execution.send_agv(kit, kit.plan.dest_tray_id)
                            if(success_agv_cmd):
                                kit.kit_plan.working_agv.release()  

    def execute(self):
        self.start_time = rospy.Time.now()
        while not self.isFinished():
            part_plan = self.get_next_part_plan()
            if part_plan is None:
                rospy.loginfo("[Scheduler] No possible Part Plan yet...")
                rospy.loginfo("[Scheduler] Sleeping...")
                rospy.sleep(2)
            else:
                execute_part = execution.ExecutePart(part_plan)
                status = execute_part.execute()
                rospy.loginfo("\n\n[Scheduler] execute_part.execute = {}\n\n".format(status))
                # part deposited successfully at tray
                if status:
                    part_plan.part.set_done()
                    rospy.loginfo("[Scheduler] Part from kit deposited successfully - {}".format(part_plan))
                    
                    if part_plan.part.parent_kit.get_status() == order_utils.Status.DONE:
                        rospy.loginfo("[Scheduler] Kit completed successfully")
                        rospy.loginfo("[Scheduler] Sending AGV")
                        success_agv_cmd = execution.send_agv(part_plan.part.parent_kit, part_plan.dest_tray_id)
                        if(success_agv_cmd):
                            part_plan.kit_plan.working_agv.release()
                        else:
                            rospy.loginfo("[Scheduler] Not Comp Implemented Yet - Could not send Agv Cmd")  
                            attempt_max = 3
                            attempt = 0
                            while  attempt <  attempt_max and not success_agv_cmd:
                                success_agv_cmd = execution.send_agv(part_plan.part.parent_kit, part_plan.dest_tray_id)
                            if(success_agv_cmd):
                                part_plan.kit_plan.working_agv.release()
                else:
                    
                    if(part_plan.part.failed_count > 9):
                        part_plan.part.set_done()  
                        if part_plan.part.parent_kit.get_status() == order_utils.Status.DONE:
                            rospy.logerr("[Scheduler] Kit completed in failure state")
                            rospy.loginfo("[Scheduler] Sending AGV")
                            success_agv_cmd = execution.send_agv(part_plan.part.parent_kit, part_plan.dest_tray_id)
                            if(success_agv_cmd):
                                part_plan.kit_plan.working_agv.release()
                            else:
                                rospy.loginfo("[Scheduler] Not Comp Implemented Yet - Could not send Agv Cmd")  
                                attempt_max = 3
                                attempt = 0
                                while  attempt <  attempt_max and not success_agv_cmd:
                                    success_agv_cmd = execution.send_agv(part_plan.part.parent_kit, part_plan.dest_tray_id)
                                if(success_agv_cmd):
                                    part_plan.kit_plan.working_agv.release()  
                    else:
                        rospy.logerr("\n\n[Scheduler] Part Plan failed\n\n")
                        part_plan.part.failed_count+=1 #TODO force a time before attempting this part again before a completly failure

            self.check_time_out_and_complete()
                        







    #Get non finished high priority order
    def get_non_fin_hp_order(self):
        len_order_list = len(self.order_list)       
        idx_high_priority = len_order_list-1 #Last means high priority

        while idx_high_priority >= 0:
            working_order = self.order_list[idx_high_priority] 
            #Not sure yet what to do for other states (HALTED, ERROR)
            if(working_order.get_status() is not order_utils.Status.DONE): 
                return working_order
            idx_high_priority-=1

    #If there is no available agv, return None
    def get_available_agv(self):
        for agv in self.agvs:
            if agv.is_available():
                return agv

    def get_available_agv_and_reserve(self, kit):
        for agv in self.agvs:
            if agv.is_available():
                rospy.loginfo("[Scheduler] get_available_agv_and_reserve: Available - " + str(agv))    
                agv.reserve(kit)
                return agv

    def get_plan_for_kit(self, kit):

        #rospy.logerr("[WorkingAgv] get_plan_for_kit not implemented yet")
        working_agv = self.get_available_agv_and_reserve(kit)
        while(working_agv is None): #TODO improve later. We do not actually need the agv for planning
            rospy.logerr("[Scheduler] No available AGV")
            rospy.sleep(1)
            working_agv = self.get_available_agv_and_reserve(kit)


        kit_plan = KitPlan(kit=kit, working_agv=working_agv, list_part_plan=None)
        return kit_plan


    def get_plan_for_part(self, working_part):

        part_plan = working_part.plan
        if(part_plan is not None):
            rospy.logerr("[Scheduler] get_plan_for_part working_part already has part_plan - " + str(working_part))    
            rospy.logerr("[Scheduler]: plan - )" + str(part_plan)) 
            return part_plan
        else:
            rospy.loginfo("[Scheduler] get_plan_for_part computing plan for working_part: " + str(working_part))    

        parent_kit = working_part.parent_kit

        if(working_part.time_last_chk_attempt_plan is None):
            working_part.time_last_chk_attempt_plan = rospy.Time.now()
            working_part.planning_attempts = 1
        
        elapsed_from_last = rospy.Time.now() - working_part.time_last_chk_attempt_plan
        # if(elapsed_from_last > PLAN_STEP_CNT):              
        #     working_part.planning_attempts += 1  
        working_part.time_last_chk_attempt_plan = rospy.Time.now()
            #rospy.logerr("\n\n[Scheduler] working_part:{} \n\n planning_attempts: {}\n\n time_last_chk_attempt_plan:{}\n\n".format(working_part, working_part.planning_attempts, working_part.time_last_chk_attempt_plan))    
            

        if(working_part.failed_count > MAX_PLAN_ERR or working_part.planning_attempts> MAX_PLAN_STEP_CNT):
            rospy.logerr("\n##############\n\n###########\n[Scheduler]: get_plan_for_part failed PERMANENTLY\nreason:failed_count > MAX_PLAN_STEP_CNT: {}/{}\n\n\n\n".format(working_part.failed_count,MAX_PLAN_STEP_CNT)) 
            pp = PartPlan(part = working_part, 
                            pick_piece=PickPiece(PickPlaces.FAIL, None, None), kit_plan =  parent_kit.plan)
            
            return pp


        kit_plan =  parent_kit.plan
        if(kit_plan is None):
            rospy.loginfo("[Scheduler] get_plan_for_part computing kit_plan: " + str(working_part))    
            kit_plan = self.get_plan_for_kit(parent_kit)

            if(kit_plan is None):
                rospy.logerr("[Scheduler] get_plan_for_part kit_plan still None")
                return
            else:
                parent_kit.plan = kit_plan    


        part_place = self.check_part_origin(working_part.part_type)
        rospy.loginfo("[Scheduler]: part_place" + str(part_place))

        part_plan = PartPlan(part = working_part, 
                            pick_piece=None, kit_plan = kit_plan)

        pick_piece = None
        for init_id in part_place:
            # part will be spawned at belt
            if init_id.unit_id == "belt":
                # checking if part is available at belt
                cam_id, part_id = global_vars.tf_manager.find_part_name(part_name=working_part.part_type, sub_dad=ORIGN_CAMERA['belt']+"_frame")
                if len(cam_id) > 0 and len(part_id) > 0:
                    rospy.loginfo("[Scheduler]: PART: {} CAMID: {} AVAILABLE AT BELT".format(part_id, cam_id))
                    pick_piece = PickPiece(PickPlaces.BELT, None, None)
                    break

            elif "bin" in init_id.unit_id:
                rospy.loginfo("[Scheduler]: PART " + working_part.part_type + " AVAILABLE AT BIN")
                pick_piece = PickPiece(PickPlaces.ANY_BIN, None, None)
                # break Without breaking herethe for loop will attemp to find parts from the belt
        
        
        if pick_piece is not None:
            part_plan = PartPlan(part = working_part, 
                            pick_piece=pick_piece, kit_plan = kit_plan)
            working_part.planning_attempts = 0

        else:
            working_part.planning_attempts+=1

        rospy.loginfo("[Scheduler]: " + str(part_plan))    

        return part_plan



    def get_next_part_plan(self):
        # TODO: improve the part select
        len_order_list = len(self.order_list)

        if(len_order_list == 0):
            rospy.loginfo("[Scheduler] No Orders yet...")   
            return

        working_order = self.get_non_fin_hp_order()
        if(working_order is None):
            rospy.loginfo("[Scheduler] There are no unfinished orders") 
            return

        working_kit = self.get_frs_non_fin_kit_from_ord(working_order)
        if(working_kit is None):
            rospy.logerr("[Scheduler] There are no unfinished kit but the order is not finished")
            rospy.logerr(working_order.get_full_repr()) 
            return


        parts_number = len(working_kit.parts)
        idx_parts = 0
        found_part_ok = False
        last_non_pu_plan = None
        use_belt = False
        part_plan = None
        while(idx_parts < parts_number):
            working_part = working_kit.parts[idx_parts]            
            if(working_part.get_status() is not order_utils.Status.DONE):
                rospy.loginfo("\n[Scheduler] Looking for plan for: {}\n".format(working_part))
                part_plan_t = self.get_plan_for_part(working_part)
                if(part_plan_t is not None and part_plan_t.pick_piece is not None):
                    found_part_ok = True
                    part_plan = part_plan_t                    
                    if(part_plan_t.pick_piece.origin is PickPlaces.BELT):	
                        use_belt = True
                        break
                    elif("pulley" not in working_part.part_type):
                        last_non_pu_plan = part_plan_t
            idx_parts+=1

        # working_part = self.get_frs_non_fin_part_from_kit(working_kit)
        # if(working_part is None):
        #     rospy.logerr("[Scheduler] There are no unfinished part but the kit is not finished")
        #     rospy.logerr(working_order.get_full_repr()) 
        #     return

        # part_plan = self.get_plan_for_part(working_part)

        if(found_part_ok):
            if(not use_belt and last_non_pu_plan is not None):                
                last_non_pu_plan.part.plan = last_non_pu_plan
                return last_non_pu_plan
            else:   
                part_plan.part.plan = part_plan                
                return part_plan
        else:
            return
        
        

    #Get first non finished part from kit
    def get_frs_non_fin_part_from_kit(self, kit):
        for part in kit.parts:
            #Not sure yet what to do for other states (HALTED, ERROR)
            if(part.get_status() is not order_utils.Status.DONE):
                return part
        


    #Get first non finished kit from order
    def get_frs_non_fin_kit_from_ord(self, order):
        for kit in order.kits:
            #Not sure yet what to do for other states (HALTED, ERROR)
            if(kit.get_status() is not order_utils.Status.DONE):
                return kit


    #ask ariac service where the part is storaged
    def check_part_origin(self, part_type):
        service_name = "/ariac/material_locations"
        rospy.wait_for_service(service_name)
        try:
            material_location = rospy.ServiceProxy(
                service_name, GetMaterialLocations)
            location = material_location(part_type)
            return location.storage_units
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to get the material location: %s" % exc)
            return []