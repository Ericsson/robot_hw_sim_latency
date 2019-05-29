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


import rospy, time, roslib
import numpy as np

from gprt import global_vars, competition, arm_actions




def main():
    global_vars.init()
    np.set_printoptions(precision=3)
    rospy.init_node("gprt_node")


    comp_class = competition.Competition()
    arm_actions.init()

    competition.init_global_vars(comp_class)
    competition.connect_callbacks(comp_class)

    rospy.loginfo('\n\nInitial competition state: {} \n\n'.format(comp_class.current_comp_state))

    rospy.loginfo("Setup complete. Sleeping a bit (services, callbacks, states)")
    rospy.sleep(3.0)
    #TODO Check if we really need to send to initial pos
    #only after and order is received.
    arm_actions.go_to_initial_position()
    rospy.loginfo("Set initial position before starting")
    rospy.sleep(2.0)
    competition.start_competition()


    competition.setup_sensors(comp_class)

    # waiting order arrives
    while comp_class.current_comp_state is None:
        rospy.loginfo_throttle(5, "No orders yet...")        
        rospy.sleep(0.1)


    rospy.loginfo("\n\nCompetition state after start: " + str(comp_class.current_comp_state))

    while "go" not in comp_class.current_comp_state:
        rospy.loginfo("\nCompetition still not in go state. Curr state: " + str(comp_class.current_comp_state))
        rospy.sleep(0.05)

    rospy.loginfo('\n\nGO\n\n')

    comp_class.start_plan_and_execute()

    


if __name__ == '__main__':
    main()
