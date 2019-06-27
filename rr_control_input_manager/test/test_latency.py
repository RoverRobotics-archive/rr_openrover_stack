#!/usr/bin/env python
from __future__ import print_function

PKG = 'rr_control_input_manager'
import roslib

roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import os
import sys
import rospkg
import rospy

rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path(PKG), 'scripts'))
import unittest
import rosunit
from geometry_msgs.msg import Twist, TwistStamped
import numpy as np


class TestLatency(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestLatency, self).__init__(*args, **kwargs)
        rospy.init_node('test_control_input_manager')
        self.trials = 1000
        self.tests = [method for method in list(TestLatency.__dict__) if 'test_' in method]
        self.message_recieved = dict(zip(self.tests, [False] * len(self.tests)))
        self.message_stamp = dict(zip(self.tests, [None] * len(self.tests)))
        self.msg_latency = dict(zip(self.tests, [np.zeros(self.trials)] * len(self.tests)))
        self.msg = dict(zip(self.tests, [None] * len(self.tests)))

    def listener(self, data, test):
        self.message_stamp[test] = rospy.Time.now()
        self.message_recieved[test] = True
        self.msg[test] = data

    def reset_message_info(self, test):
        self.msg[test] = None
        self.message_recieved[test] = False
        self.message_stamp[test] = None

    def generate_twist(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = rx
        msg.angular.y = ry
        msg.angular.z = rz

        return msg

    def generate_twist_stamped(self, time, seq=0, frame_id='', x=0, y=0, z=0, ax=0, ay=0, az=0):
        msg = TwistStamped()
        msg.header.seq = seq
        msg.header.stamp = time
        msg.header.frame_id = frame_id
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        msg.twist.angular.x = ax
        msg.twist.angular.y = ay
        msg.twist.angular.z = az

        return msg

    def compare_twist_stamped(self, msg1, msg2):
        result = msg1.header.stamp == msg2.header.stamp \
                 and msg1.header.frame_id == msg2.header.frame_id \
                 and msg1.twist == msg2.twist

        return result

    def test_twist_stamped_latency(self):
        TEST = 'test_twist_stamped_latency'
        rate = rospy.Rate(100)

        sub = rospy.Subscriber('/test/outputF', TwistStamped, self.listener, TEST, queue_size=1)
        pub = rospy.Publisher('/test/inputF', TwistStamped, queue_size=1)

        rospy.sleep(1)

        for i in range(self.trials):
            msg = self.generate_twist_stamped(rospy.Time.now(), frame_id=str(i))
            pub.publish(msg)

            timeout = 0
            while timeout < 1000 and not self.message_recieved[TEST]:
                rate.sleep()
                timeout += 1

            try:
                self.assertTrue(self.compare_twist_stamped(msg, self.msg[TEST]))
                self.msg_latency[TEST][i] = (self.message_stamp[TEST] - msg.header.stamp).to_sec()
                self.reset_message_info(TEST)
            except self.failureException as e:
                self.reset_message_info(TEST)
                pub.unregister()
                sub.unregister()
                raise self.failureException(e)
        self.assertLess(np.max(self.msg_latency[TEST]), 0.05)
        self.assertGreater(np.min(self.msg_latency[TEST]), 0.0)
        self.assertLess(np.mean(self.msg_latency[TEST]), 0.05)
        pub.unregister()
        sub.unregister()

    def test_twist_unstamped_latency(self):
        TEST = 'test_twist_unstamped_latency'
        rate = rospy.Rate(100)

        sub = rospy.Subscriber('/test/outputA', Twist, self.listener, TEST, queue_size=1)
        pub = rospy.Publisher('/test/inputA', TwistStamped, queue_size=1)

        rospy.sleep(1)

        for i in range(self.trials):
            msg = self.generate_twist_stamped(rospy.Time.now(), frame_id=str(i))
            pub.publish(msg)

            timeout = 0
            while timeout < 1000 and not self.message_recieved[TEST]:
                rate.sleep()
                timeout += 1

            try:
                self.assertEqual(msg.twist, self.msg[TEST])
                self.msg_latency[TEST][i] = (self.message_stamp[TEST] - msg.header.stamp).to_sec()
                self.reset_message_info(TEST)
            except self.failureException as e:
                self.reset_message_info(TEST)
                pub.unregister()
                sub.unregister()
                raise self.failureException(e)

        self.assertLess(np.max(self.msg_latency[TEST]), 0.05)
        self.assertGreater(np.min(self.msg_latency[TEST]), 0.0)
        self.assertLess(np.mean(self.msg_latency[TEST]), 0.05)
        pub.unregister()
        sub.unregister()


rosunit.unitrun(PKG, 'test_control_input_manager_latency', TestLatency)
