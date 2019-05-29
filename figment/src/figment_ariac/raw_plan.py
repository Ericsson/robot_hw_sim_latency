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

from gprt import control
import rospy
import time
import numpy as np
import roslib
from gprt import actions
from gprt import global_vars



def main():
    global_vars.init()
    np.set_printoptions(precision=3)
    rospy.init_node("gprt_node")

    comp_class = control.Gprt()

    control.init_global_vars(comp_class)
    control.connect_callbacks(comp_class)

    rospy.loginfo("Setup complete. Sleeping before starting (services and callbacks)")
    rospy.sleep(5.0)
    control.start_competition()

    # if not comp_class.has_been_zeroed:
    #     comp_class.has_been_zeroed = True
    #     rospy.loginfo("Sending arm to zero joint positions...")
    #     comp_class.send_arm_to_state([0] * len(comp_class.arm_joint_names))

    control.setup_sensors(comp_class)

    # waiting order arrives
    while comp_class.current_comp_state is None:
        rospy.loginfo("\n\nNo orders yet. Sleeping while orders arribe")
        rospy.sleep(1.0)

    comp_class.go_to_initial_position()

    rospy.loginfo("\n\nInitial competition state: " + str(comp_class.current_comp_state))
 
    #In some cases, non-deterministic, 
    #the program reaches this point while the competition is still in init state
    #TODO: chek if there is a better way to deal with this.
    while "go" not in comp_class.current_comp_state:
        rospy.loginfo("\nCompetition still no in Go state: " + str(comp_class.current_comp_state))


    while "go" in comp_class.current_comp_state:

        if len(comp_class.actions_tray1) > 0 or len(comp_class.actions_tray2) > 0:
            # rospy.loginfo("\n\n\n\nProcessing ORDER  LenTray1 = "
            #               + str(len(comp_class.actions_tray1)) + "LenTray2 = "
            #               + str(len(comp_class.actions_tray2)) + "\n\n\n\n")
            """
            The second tray will be the high priority one,
            and every order that comes might be placed in
            the first one. When a new order arrives it will
            be placed in the second one and be executed unless
            we are finishing the pick and place of a part.
            """
            # if there is elements in the second tray and the action to be executed
            # in the first tray is check where the material is we go to the second
            # tray
            action_list = None
            trayid = 0
            kit_object = None
            started_tray1 = False
            started_tray2 = False
            if len(comp_class.actions_tray1) > 0:
                action_list = comp_class.actions_tray1
                trayid = 1
                started_tray1 = True
            elif len(comp_class.actions_tray2) > 0:
                action_list = comp_class.actions_tray2
                trayid = 2
                started_tray2 = True

            # rospy.loginfo("Action List Start Len: " + str(len(action_list)))

            # DEBUG
            kit_object_list = []

            for x in action_list:
                if(isinstance(x, actions.ActionMaterialLocation)):
                    kit_object_list.append(x.kit_object.type)

            # rospy.loginfo("Action List Objects: " + str(kit_object_list))

            action = action_list.pop()

            # rospy.loginfo("Action List Middle Len: " + str(len(action_list)))

            if isinstance(action, actions.ActionMaterialLocation):
                result = action.execute_action()
                rospy.loginfo("Material checking result: " + str(result))
                # the type is in the scenario so we need to create the next
                # actions
                if len(result) > 0:
                    rospy.logerr("Len action_list : " + str(len(action_list)))

                    # send avg
                    if len(action_list) > 0 and not isinstance(action_list[0], actions.ActionMaterialLocation):
                        rospy.logerr("#######\n\n\n GOING TO FINISH \n\n\n\" ")
                        comp_class.process_order_into_actions(
                            comp_class, action, action_list, action.kit_object, trayid, result, action.kit_type, True, True)
                    else:
                        rospy.logerr("#######\n\n\n NORMAL \n\n\n\" ")
                        comp_class.process_order_into_actions(
                            comp_class, action, action_list, action.kit_object, trayid, result, action.kit_type, False, True)

                else:
                    # reinserting the action in the and of the stack
                    action_list.insert(0, action)
            elif isinstance(action, actions.ActionCameraTF):
                rospy.loginfo("Executing action " + action.__class__.__name__)
                result = action.execute_action()
                # calculating object position
                if result is not None:
                    pos, rot = result
                    rospy.loginfo("Position Calculated: [" + str(pos[0]) + ", " + str(pos[1]) + ", " + str(pos[2]) + "]")
                # if object not in tf buffer try again
                else:
                    action_list.append(action)
                

            else:
                rospy.loginfo("Executing action " + action.__class__.__name__)
                result = action.execute_action()

            # if(started_tray1 and len(comp_class.actions_tray1) == 0):
            #     rospy.loginfo("[process_order_into_actions] - Send Order")
            #     send_agv = actions.ActionSendAgv(1, action.action.kit_type)
            #     send_agv.execute_action()

            # if(started_tray2 and len(comp_class.actions_tray2) == 0):
            #     rospy.loginfo("[process_order_into_actions] - Send Order")
            #     send_agv = actions.ActionSendAgv(2, action.action.kit_type)
            #     send_agv.execute_action()

            rospy.loginfo("Action List End Len: " + str(len(action_list)))

            # when send an agv a new order must be processed
        else:
            rospy.sleep(0.1)


if __name__ == '__main__':
    main()
