#!/usr/bin/env python
import rospy
import unittest
import rosunit
from geometry_msgs.msg import Twist, TwistStamped
import numpy as np

PKG = 'rr_control_input_manager'


class TestLatency(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestLatency, self).__init__(*args, **kwargs)
        self.trials = 1000
        self.message_received = False
        self.message_stamp = None
        self.msg_latency = np.zeros(self.trials)
        self.msg = None
        self.pub = None
        self.sub = None

    def setUp(self):
        test_method_name = self._testMethodName
        subscribers = {'test_twist_unstamped_latency': ('/test/outputA', Twist),
                       'test_twist_stamped_latency': ('/test/outputF', TwistStamped)
                       }
        publishers = {'test_twist_unstamped_latency': ('/test/inputA', TwistStamped),
                      'test_twist_stamped_latency': ('/test/inputF', TwistStamped),
                      }

        if test_method_name in subscribers.keys():
            self.sub = rospy.Subscriber(subscribers[test_method_name][0],
                                        subscribers[test_method_name][1],
                                        self.listener,
                                        queue_size=10)
        if test_method_name in publishers.keys():
            self.pub = rospy.Publisher(publishers[test_method_name][0],
                                       publishers[test_method_name][1],
                                       queue_size=1)

        # Allow time for the publisher and subscriber to properly connect to endpoints
        rospy.sleep(1)

    def tearDown(self):
        if self.sub:
            self.sub.unregister()
        if self.pub:
            self.pub.unregister()

    def listener(self, data):
        self.message_stamp = rospy.Time.now()
        self.message_received = True
        self.msg = data

    def reset_message_info(self):
        self.msg = None
        self.message_received = False
        self.message_stamp = None

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
        rate = rospy.Rate(100)

        for i in range(self.trials):
            msg = self.generate_twist_stamped(rospy.Time.now(), frame_id=str(i))
            self.pub.publish(msg)

            timeout = 0
            while timeout < 1000 and not self.message_received:
                rate.sleep()
                timeout += 1

            self.assertTrue(self.compare_twist_stamped(msg, self.msg))
            self.msg_latency[i] = (self.message_stamp - msg.header.stamp).to_sec()
            self.reset_message_info()

        self.assertLess(np.max(self.msg_latency), 0.05)
        self.assertGreater(np.min(self.msg_latency), 0.0)
        self.assertLess(np.mean(self.msg_latency), 0.05)

    def test_twist_unstamped_latency(self):
        rate = rospy.Rate(100)

        for i in range(self.trials):
            msg = self.generate_twist_stamped(rospy.Time.now(), frame_id=str(i))
            self.pub.publish(msg)

            timeout = 0
            while timeout < 1000 and not self.message_received:
                rate.sleep()
                timeout += 1

            self.assertEqual(msg.twist, self.msg)
            self.msg_latency[i] = (self.message_stamp - msg.header.stamp).to_sec()
            self.reset_message_info()

        self.assertLess(np.max(self.msg_latency), 0.05)
        self.assertGreater(np.min(self.msg_latency), 0.0)
        self.assertLess(np.mean(self.msg_latency), 0.05)


rospy.init_node('test_control_input_manager_latency_test')
rosunit.unitrun(PKG, 'test_control_input_manager_latency', TestLatency)
