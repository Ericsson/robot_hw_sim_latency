#!/usr/bin/env python
from enum import Enum

import rospy


    
class Status(Enum):
    INIT = 1
    EXECUTING = 2
    HALTED = 3
    DONE = 4
    ERROR = 5


class Order:


    def __init__(self, ariac_order_msg):
    	self.order_id = ariac_order_msg.order_id

    	self.kits = []
    	self.append_kits(self.kits, ariac_order_msg.kits)

    	self.state = Status.INIT
        self.time_started = None


    def __str__(self):

        return 'Order id:{}; #kits:{}; state:{}'.format(
        	self.order_id, self.get_number_kits(), self.get_status())

    def __repr__(self):
        return 'Order id:{}; #kits:{}; state:{}'.format(
            self.order_id, self.get_number_kits(), self.get_status())

    def get_full_repr(self):
        header = 'Order id:{}; #kits:{}; state:{}'.format(
            self.order_id, self.get_number_kits(), self.get_status())

        str_kits = '\n'
        for kit in self.kits:
            str_kits = str_kits + "##" + kit.get_full_repr() + "\n"  

        return header + str_kits  
        
    def append_kits(self, array_kits, ariac_kits_msg):
    	start_id = len(array_kits)

    	for ariac_kit_msg in ariac_kits_msg:
    		kit_id = self.order_id + "::kid_" + str(start_id)
    		start_id+=1
    		kit = Kit(kit_id=kit_id, parent_order=self, 
    			ariac_kit_msg = ariac_kit_msg)
    		array_kits.append(kit)



    def get_number_kits(self):
        return len(self.kits)

    def get_status(self):
    	return self.state

    def set_done(self):
        self.state = Status.DONE
        return True

    def get_kit_by_id(self, kit_id):
        idx = 0
        len_kits = len(self.kits)

        while(idx < len_kits):
            kit = self.kits[idx]  

            if(kit_id == kit.kit_id):
                return idx, kit

            idx+=1    

    
    def deep_check_done(self):
        for kit in self.kits:
            if kit.get_status() is not Status.DONE:
                return False
        return True


    def process_kit_done(self, kit):
        if(self.deep_check_done()):
            self.set_done()


    def reset(self):
    	for kit in self.kits:
    		kit.reset()
    	self.state = Status.INIT

    def set_time_started_if_not_already(self, time):
        if(self.time_started is None):
            rospy.loginfo("[Scheduler] Order: {}; Started at: {}".format(self, time))  
            self.time_started = time



class Kit:

    def __init__(self, kit_id, parent_order, ariac_kit_msg):
    	self.kit_id = kit_id
    	self.parent_order = parent_order
    	self.kit_type = ariac_kit_msg.kit_type
    	self.parts = []
    	self.append_parts(self.parts, ariac_kit_msg.objects)
    	self.state = Status.INIT
        self.plan = None
        self.failed_count = 0
        self.time_started = None

    def __str__(self):

        return 'Kit id:{}; kit_type:{}; #parts:{}; state:{}'.format(
        	self.kit_id, self.kit_type, self.get_number_parts(), 
        	self.get_status())

    def __repr__(self):
        return 'Kit id:{}; kit_type:{}; #parts:{}; state:{}'.format(
            self.kit_id, self.kit_type, self.get_number_parts(), 
            self.get_status())

    def get_full_repr(self):
        header = 'Kit id: {}; kit_type:{}; #parts: {}; state: {}'.format(
            self.kit_id, self.kit_type, self.get_number_parts(), 
            self.get_status())

        str_parts = '\n'
        for part in self.parts:
            str_parts = str_parts + "####" + str(part) + '\n'  

        return header + str_parts          

    def append_parts(self, array_parts, ariac_kitObjects_msg):
    	start_id = len(array_parts)

    	for ariac_kitObject_msg in ariac_kitObjects_msg:
    		part_id = self.kit_id + "::pid_" + str(start_id)
    		start_id+=1
    		part = Part(part_id = part_id, 
    			parent_kit = self, 
    			ariac_kitObject_msg = ariac_kitObject_msg)
    		array_parts.append(part)
        

    def get_number_parts(self):
        return len(self.parts)

    def get_status(self):
    	return self.state

    def set_done(self):
        self.state = Status.DONE
        self.parent_order.process_kit_done(self)
        return True

    def deep_check_done(self):
        for part in self.parts:
            if part.get_status() is not Status.DONE:
                return False
        return True

    def process_part_done(self, part):
        if(self.deep_check_done()):
            self.set_done()


    def reset(self):
    	for part in self.parts:
    		part.reset()
    	self.state = Status.INIT
        self.plan = None #TODO check later

    def set_time_started_if_not_already(self, time):
        if(self.time_started is None):
            rospy.loginfo("[Scheduler] Kit: {}; Started at: {}".format(self, time))  
            self.time_started = time
            self.parent_order.set_time_started_if_not_already(time)


class Part:

    def __init__(self, part_id, parent_kit, ariac_kitObject_msg):
    	self.part_id = part_id
    	self.parent_kit = parent_kit
    	self.part_type = ariac_kitObject_msg.type
    	self.desired_pose = ariac_kitObject_msg.pose
    	self.state = Status.INIT
        self.plan = None
        self.failed_count = 0
        self.time_started = None
        self.planning_attempts = 0
        self.time_last_chk_attempt_plan = None


    def __str__(self):

        return 'Part id:{}; type:{}; desired_position: {}|{}|{}; desired_orientation:{}|{}|{}; state:{}'.format(
        	self.part_id, self.part_type, self.desired_pose.position.x, 
            self.desired_pose.position.y, self.desired_pose.position.z,
            self.desired_pose.orientation.x, self.desired_pose.orientation.y, 
            self.desired_pose.orientation.z, 
        	self.get_status())

    def __repr__(self):
        return 'Part id:{}; type:{}; desired_position: {}|{}|{}; desired_orientation:{}|{}|{}; state:{}'.format(
            self.part_id, self.part_type, self.desired_pose.position.x, 
            self.desired_pose.position.y, self.desired_pose.position.z,
            self.desired_pose.orientation.x, self.desired_pose.orientation.y, 
            self.desired_pose.orientation.z, 
            self.get_status())
    

    def get_status(self):
    	return self.state

    def reset(self):
    	self.state = Status.INIT
        self.plan = None #TODO check later

    def set_done(self):
        self.state = Status.DONE
        self.parent_kit.process_part_done(self)
        return True

    def set_time_started_if_not_already(self, time):
        if(self.time_started is None):
            rospy.loginfo("[Scheduler] Part: {}; Started at: {}".format(self, time))  
            self.time_started = time
            self.parent_kit.set_time_started_if_not_already(time)






