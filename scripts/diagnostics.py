#!/usr/bin/env python

import rospy
import diagnostic_updater
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rr_openrover_basic.msg import RawRrOpenroverBasicSlowRateData, RawRrOpenroverBasicMedRateData

class rover_diagnostic():

    def __init__(self):
        rospy.Subscriber("/raw_slow_rate_data", RawRrOpenroverBasicSlowRateData, self.slow_data_cb)
        rospy.Subscriber("/raw_med_rate_data", RawRrOpenroverBasicMedRateData, self.med_data_cb)
        self.pub = rospy.Publisher('/inorbit/custom_data/0', String, queue_size=5)

    def slow_data_cb(self, data):
        warn_msg = "Battery Levels [" + str(data.reg_robot_rel_soc_a) + ", " + str(data.reg_robot_rel_soc_b) + "]    Motor temps [" + str(data.reg_motor_temp_left) + ", " + str(data.reg_motor_temp_right) + "]"
#        rospy.logwarn_throttle(60,warn_msg)

        self.pub.publish("Battery Cell 1 SOC=" + str(data.reg_robot_rel_soc_a))
        self.pub.publish("Battery Cell 2 SOC=" + str(data.reg_robot_rel_soc_b))
        self.pub.publish("Left Motor Temp=" + str(data.reg_motor_temp_left))
        self.pub.publish("Right Motor Temp=" + str(data.reg_motor_temp_right))
        self.pub.publish("Battery Status A=" + str(data.battery_status_a))
        self.pub.publish("Battery Status B=" + str(data.battery_status_b))
        self.pub.publish("Battery Mode A=" + str(data.battery_mode_a))
        self.pub.publish("Battery Mode B=" + str(data.battery_mode_b))
        self.pub.publish("Battery Temp A=" + str(data.battery_temp_a))
        self.pub.publish("Power Bat Voltage A=" + str(data.reg_power_bat_voltage_a))
        self.pub.publish("Power Bat Voltage B=" + str(data.reg_power_bat_voltage_b))
        self.pub.publish("Battery Voltage A=" + str(data.battery_voltage_a))
        self.pub.publish("Battery Voltage B=" + str(data.battery_voltage_b))

    def med_data_cb(self, data):
        self.pub.publish("Motor Feedback Current Left=" + str(data.reg_motor_fb_current_left))
        self.pub.publish("Motor Feedback Current Right=" + str(data.reg_motor_fb_current_right))
        self.pub.publish("Power A Current=" + str(data.reg_power_a_current))
        self.pub.publish("Power B Current=" + str(data.reg_power_b_current))
        self.pub.publish("Battery Current A=" + str(data.battery_current_a))
        self.pub.publish("Battery Current B=" + str(data.battery_current_b))

if __name__ == '__main__':
    rospy.loginfo("Starting node")
    rospy.init_node('openrover_diagnostics_node')

    my_diagnostic = rover_diagnostic()
    rospy.spin()

