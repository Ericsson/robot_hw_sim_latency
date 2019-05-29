#!/usr/bin/env python

import rospy
import time
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties

from rosnode import get_node_names

if __name__ == "__main__":
    rospy.init_node('unpausesimulation', anonymous=True)
    print "\033[0;32mwaiting for service /gazebo/unpause_physics"
    rospy.wait_for_service('/gazebo/unpause_physics')
    get_physics_client=rospy.ServiceProxy('/gazebo/get_physics_properties',GetPhysicsProperties)
    set_physics_client=rospy.ServiceProxy('/gazebo/set_physics_properties',SetPhysicsProperties)
    get_prop=get_physics_client()
    gravity=get_prop.gravity
    gravity.z=0.0
    set_physics_client(time_step=get_prop.time_step, max_update_rate=get_prop.max_update_rate, gravity=gravity, ode_config=get_prop.ode_config)
    print "\033[0;32mwaiting for /ur5_vel_control node"
    canstart=False
    while not canstart:
		if(rospy.is_shutdown()):
			exit()
		nodes=get_node_names()
		if "/ur5_vel_control" in nodes:
			canstart=True
		time.sleep(0.5)

    print "\033[0;32mwaiting for 10 secs"
    time.sleep(10.0)
    print "\033[0;32munpausing simulation"
    unpause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
    unpause_physics_client()
    set_physics_client(time_step=get_prop.time_step, max_update_rate=get_prop.max_update_rate, gravity=get_prop.gravity, ode_config=get_prop.ode_config)


