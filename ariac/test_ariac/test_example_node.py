#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from ariac_example import ariac_example
from std_msgs.msg import Float32
import rospy
import rostest


class ExampleNodeTester(unittest.TestCase):

    def comp_score_callback(self, msg):
        self.current_comp_score = msg.data

    def prepare_tester(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)

        self.current_comp_score = None
        self.comp_state_sub = rospy.Subscriber(
            "/ariac/current_score", Float32, self.comp_score_callback)

        # Pre-defined initial pose because sometimes the arm starts "droopy"
        self._send_arm_to_initial_pose()

    def test(self):
        self.prepare_tester()
        self._test_send_arm_to_zero_state()

        # Starting the competition will cause parts from the order to be spawned on AGV1
        self._test_start_comp()
        time.sleep(1.0)
        self._test_order_reception()

        self._test_agv_control()
        time.sleep(5.0)
        self._test_comp_end()

    def _test_start_comp(self):
        success = ariac_example.start_competition()
        self.assertTrue(success, 'Failed to start the competition')
        time.sleep(0.5)
        self.assertTrue(
            self.comp_class.current_comp_state == 'go', 'Competition not in "go" state')

    def _test_order_reception(self):
        self.assertEqual(len(self.comp_class.received_orders), 1)

    def _send_arm_to_initial_pose(self):
        positions = [1.51, 0.0, -1.12, 3.14, 3.77, -1.51, 0.0]
        self.comp_class.send_arm_to_state(positions)
        time.sleep(1.0)

    def _test_send_arm_to_zero_state(self):
        self.comp_class.send_arm_to_state([0] * len(self.comp_class.arm_joint_names))
        # This can be slow if there are a lot of models in the environment
        time.sleep(5.0)
        error = 0
        for position in self.comp_class.current_joint_state.position:
            error += abs(position - 0.0)
        self.assertTrue(error < 0.5, 'Arm was not properly sent to zero state')

    def _test_agv_control(self, index=1, kit_id='order_0_kit_0'):
        success = ariac_example.control_agv(index, kit_id)
        self.assertTrue(success, 'Failed to control AGV')

    def _test_comp_end(self):
        num_received_orders = len(self.comp_class.received_orders)
        num_kits = len(self.comp_class.received_orders[0].kits)
        if num_received_orders == 1 and num_kits == 1:
            self.assertTrue(
                self.comp_class.current_comp_state == 'done', 'Competition not in "done" state')
        else:
            # If there were more kits expected, the order won't be done
            self.assertTrue(
                self.comp_class.current_comp_state == 'go', 'Competition not in "go" state')
        num_parts_in_order = len(self.comp_class.received_orders[0].kits[0].objects)
        self.assertTrue(
            # Expect to have a point for each part, the all parts bonus, and a point for each part's pose
            self.current_comp_score == 3 * num_parts_in_order,
            'Something went wrong in the scoring. Current score: ' + str(self.current_comp_score))


if __name__ == '__main__':
    rospy.init_node('test_example_node', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('osrf_gear', 'test_example_node', ExampleNodeTester, sys.argv)
