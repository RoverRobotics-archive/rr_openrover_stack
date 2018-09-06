#!/usr/bin/env python

import rospy
import diagnostic_updater
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rr_openrover_basic.msg import RawRrOpenroverBasicSlowRateData

class rover_diagnostic():

    def __init__(self):
        rospy.Subscriber("/rr_openrover_basic/raw_slow_rate_data", RawRrOpenroverBasicSlowRateData, self.slow_data_cb)
        self.pub = rospy.Publisher('/inorbit/custom_data/0', String, queue_size=5)

    def slow_data_cb(self, data):
        warn_msg = "Battery Levels [" + str(data.reg_robot_rel_soc_a) + ", " + str(data.reg_robot_rel_soc_b) + "]    Motor temps [" + str(data.reg_motor_temp_left) + ", " + str(data.reg_motor_temp_right) + "]"
#        rospy.logwarn_throttle(60,warn_msg)

        # Initialize Key Value message
        msg = String()

        # Publish battery cell 1 SOC
        msg.data = "Battery Cell 1 SOC=" + str(data.reg_robot_rel_soc_a)
        self.pub.publish(msg)

        # Publish battery cell 2 SOC
        msg.data = "Battery Cell 2 SOC=" + str(data.reg_robot_rel_soc_b)
        self.pub.publish(msg)

        # Publish motor 1 temp 
        msg.data = "Left Motor Temp=" + str(data.reg_motor_temp_left)
        self.pub.publish(msg)

        # Publish motor 2 temp
        msg.data = "Right Motor Temp=" + str(data.reg_motor_temp_right)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.loginfo("Starting node")
    rospy.init_node('openrover_diagnostics_node')

    my_diagnostic = rover_diagnostic()
    rospy.spin()

