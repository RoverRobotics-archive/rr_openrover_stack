#!/usr/bin/env python
import rospy
import unittest
import rosunit
from geometry_msgs.msg import Twist, TwistStamped

PKG = 'rr_control_input_manager'


class TestTwistCommand(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestTwistCommand, self).__init__(*args, **kwargs)
        self.message_received = False
        self.message_stamp = None
        self.msg = None
        self.pub = None
        self.sub = None

    def setUp(self):
        test_method_name = self._testMethodName
        subscribers = {'test_manager_unstamped': ('/test/outputA', Twist),
                       'test_manager_stamped': ('/test/outputF', TwistStamped),
                       'test_latency_management_zero': ('/test/outputG', Twist),
                       'test_latency_management_one': ('/test/outputH', Twist),
                       'test_latency_management_less_than_zero': ('/test/outputC', Twist)
                       }
        publishers = {'test_manager_unstamped': ('/test/inputA', TwistStamped),
                      'test_manager_stamped': ('/test/inputF', TwistStamped),
                      'test_latency_management_zero': ('/test/inputG', TwistStamped),
                      'test_latency_management_one': ('/test/inputH', TwistStamped),
                      'test_latency_management_less_than_zero': ('/test/inputC', TwistStamped)
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

        # rospy.signal_shutdown('Test {test_name} completed'.format(test_name=self._testMethodName))

    def listener(self, data):
        self.message_stamp = rospy.Time.now()
        self.message_received = True
        self.msg = data

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

    def twist_stamped_eq(self, msg1, msg2):
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
        rate = rospy.Rate(100)

        msg = self.generate_twist_stamped(rospy.Time.now())
        self.pub.publish(msg)

        for timeout in range(1000):
            if self.message_received:
                break
            rate.sleep()

        self.assertEqual(msg.twist, self.msg)

    def test_manager_stamped(self):
        TEST = 'test_manager_stamped'
        rate = rospy.Rate(100)

        msg = self.generate_twist_stamped(rospy.Time.now(), frame_id=TEST)
        self.pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_received:
            rate.sleep()
            timeout += 1

        self.assertTrue(self.twist_stamped_eq(msg, self.msg))

    def test_latency_management_zero(self):
        rate = rospy.Rate(100)
        msg = self.generate_twist_stamped(rospy.Time.now())

        # This is used to simulate latency
        rospy.sleep(10)

        self.pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_received:
            rate.sleep()
            timeout += 1

        self.assertEqual(msg.twist, self.msg)

    def test_latency_management_one(self):
        rate = rospy.Rate(100)
        msg = self.generate_twist_stamped(rospy.Time.now())

        # This is used to simulate latency
        rospy.sleep(10)

        self.pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_received:
            rate.sleep()
            timeout += 1

        self.assertEqual(None, self.msg)

    def test_latency_management_less_than_zero(self):
        rate = rospy.Rate(100)
        msg = self.generate_twist_stamped(rospy.Time.now())

        # This is used to simulate latency
        rospy.sleep(10)

        self.pub.publish(msg)

        timeout = 0
        while timeout < 1000 and not self.message_received:
            rate.sleep()
            timeout += 1

        self.assertEqual(msg.twist, self.msg)


rospy.init_node('test_control_input_manager_functional_test')
rosunit.unitrun(PKG, 'test_control_input_manager_functional', TestTwistCommand)
