#!/usr/bin/env python

import rospy
import diagnostic_updater
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from rr_openrover_basic.msg import RawRrOpenroverBasicSlowRateData

class rover_diagnostic():

    def __init__(self):
        rospy.Subscriber("/raw_slow_rate_data", RawRrOpenroverBasicSlowRateData, self.slow_data_cb)
        rospy.Subscriber('/rr_openrover_basic/charging', Bool, self.openrover_charging_cb)
        self.pub = rospy.Publisher('/inorbit/custom_data/0', String, queue_size=5)

    def slow_data_cb(self, data):
        warn_msg = "Battery Levels [" + str(data.reg_robot_rel_soc_a) + ", " + str(data.reg_robot_rel_soc_b) + "]    Motor temps [" + str(data.reg_motor_temp_left) + ", " + str(data.reg_motor_temp_right) + "]"
#        rospy.logwarn_throttle(60,warn_msg)

        # Publish battery cell 1 SOC
        self.pub.publish(("Battery Cell 1 SOC=" + str(data.reg_robot_rel_soc_a)))

        # Publish battery cell 2 SOC
        self.pub.publish(("Battery Cell 2 SOC=" + str(data.reg_robot_rel_soc_b)))

        # Publish motor 1 temp 
        self.pub.publish(("Left Motor Temp=" + str(data.reg_motor_temp_left)))

        # Publish motor 2 temp
        self.pub.publish("Right Motor Temp=" + str(data.reg_motor_temp_right))

        # Publish battery_status_a
        self.pub.publish("Battery Status A=" + str(data.battery_status_a))

        # Publish battery_status_b
        self.pub.publish("Battery Status B=" + str(data.battery_status_b))

        # Publish battery_mode_a
        self.pub.publish("Battery Mode A=" + str(data.battery_mode_a))

        # Publish battery_mode_b
        self.pub.publish("Battery Mode B=" + str(data.battery_mode_b))

        # Publish battery_temp_a
        self.pub.publish("Battery Temp A=" + str(data.battery_temp_a))

        # Publish battery_temp_b
        self.pub.publish("Battery Temp B=" + str(data.battery_temp_b))

    def openrover_charging_cb(self, msg):
        self.pub.publish("Open Rover Charging=" + str(msg.data))


if __name__ == '__main__':
    rospy.loginfo("Starting node")
    rospy.init_node('openrover_diagnostics_node')

    my_diagnostic = rover_diagnostic()
    rospy.spin()

