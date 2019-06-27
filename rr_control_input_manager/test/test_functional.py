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


## A sample python unit test
class TestTwistCommand(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestTwistCommand, self).__init__(*args, **kwargs)
        rospy.init_node('test_control_input_manager')
        self.tests = [method for method in list(TestTwistCommand.__dict__) if 'test_' in method]
        self.message_recieved = dict(zip(self.tests, [False] * len(self.tests)))
        self.message_stamp = dict(zip(self.tests, [None] * len(self.tests)))
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

    def test_creates_proper_topics(self):  # only functions with 'test_'-prefix will be run!
        topics = [item[0] for item in rospy.get_published_topics()]
        self.assertIn('/test/outputA', topics)
        self.assertIn('/test/outputB', topics)
        self.assertIn('/test/outputC', topics)
        self.assertIn('/test/outputF', topics)
        self.assertIn('/test/outputG', topics)

    def test_bad_timeout(self):  # only functions with 'test_'-prefix will be run!
        topics = [item[0] for item in rospy.get_published_topics()]
        self.assertNotIn('/test/inputD', topics)
        self.assertNotIn('/test/outputD', topics)

    def test_bad_stamped(self):
        topics = [item[0] for item in rospy.get_published_topics()]
        self.assertNotIn('/test/inputE', topics)
        self.assertNotIn('/test/outputE', topics)
        print(topics)

    def test_manager_unstamped(self):
        TEST = 'test_manager_unstamped'
        rate = rospy.Rate(100)

        sub = rospy.Subscriber('/test/outputA', Twist, self.listener, TEST, queue_size=10)
        pub = rospy.Publisher('/test/inputA', TwistStamped, queue_size=10)

        rospy.sleep(1)

        msg = self.generate_twist_stamped(rospy.Time.now())
        pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_recieved[TEST]:
            rate.sleep()
            timeout += 1

        try:
            self.assertEqual(msg.twist, self.msg[TEST])
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
        except self.failureException as e:
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
            raise self.failureException(e)

    def test_manager_stamped(self):
        TEST = 'test_manager_stamped'
        rate = rospy.Rate(100)

        sub = rospy.Subscriber('/test/outputF', TwistStamped, self.listener, TEST, queue_size=10)
        pub = rospy.Publisher('/test/inputF', TwistStamped, queue_size=10)

        rospy.sleep(1)

        msg = self.generate_twist_stamped(rospy.Time.now(), frame_id=TEST)
        pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_recieved[TEST]:
            rate.sleep()
            timeout += 1

        try:
            self.assertTrue(self.compare_twist_stamped(msg, self.msg[TEST]))
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
        except self.failureException as e:
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
            raise self.failureException(e)

    def test_latency_management_zero(self):
        TEST = 'test_latency_management_zero'
        rate = rospy.Rate(100)

        sub = rospy.Subscriber('/test/outputG', Twist, self.listener, TEST, queue_size=10)
        pub = rospy.Publisher('/test/inputG', TwistStamped, queue_size=10)
        msg = self.generate_twist_stamped(rospy.Time.now())

        rospy.sleep(10)

        pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_recieved[TEST]:
            rate.sleep()
            timeout += 1

        try:
            self.assertEqual(msg.twist, self.msg[TEST])
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
        except self.failureException as e:
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
            raise self.failureException(e)

    def test_latency_management_one(self):
        TEST = 'test_latency_management_one'
        rate = rospy.Rate(100)
        sub = rospy.Subscriber('/test/outputH', Twist, self.listener, TEST, queue_size=10)
        pub = rospy.Publisher('/test/inputH', TwistStamped, queue_size=10)
        msg = self.generate_twist_stamped(rospy.Time.now())

        rospy.sleep(10)

        pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_recieved[TEST]:
            rate.sleep()
            timeout += 1

        try:
            self.assertEqual(None, self.msg[TEST])
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
        except self.failureException as e:
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
            raise self.failureException(e)

    def test_latency_management_less_than_zero(self):
        TEST = 'test_latency_management_less_than_zero'
        rate = rospy.Rate(100)

        sub = rospy.Subscriber('/test/outputC', Twist, self.listener, TEST, queue_size=10)
        pub = rospy.Publisher('/test/inputC', TwistStamped, queue_size=10)
        msg = self.generate_twist_stamped(rospy.Time.now())

        rospy.sleep(10)

        pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_recieved[TEST]:
            rate.sleep()
            timeout += 1

        try:
            self.assertEqual(msg.twist, self.msg[TEST])
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
        except self.failureException as e:
            self.reset_message_info(TEST)
            pub.unregister()
            sub.unregister()
            raise self.failureException(e)


rosunit.unitrun(PKG, 'test_control_input_manager', TestTwistCommand)
