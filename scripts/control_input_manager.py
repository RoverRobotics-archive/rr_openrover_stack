#!/usr/bin/env python

# Author: Nick Fragale
# Description: This script manages cmd_vel from multiple sources so that they don't over-ride eachother, and so that soft E-stop can works from multiple sources. 


import rospy
import time
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from actionlib_msgs.msg import GoalID

class CmdVelManager(object):

    use_joystick = True
    soft_estop = False
    local_control_lock = False
    remote_control_lock = False
    managed_control_input = TwistStamped()
    seq = 0
    def __init__(self):

        self.joystick_sub = rospy.Subscriber("/cmd_vel/joystick", TwistStamped, self.joystick_cb)
        self.move_base_sub = rospy.Subscriber("/cmd_vel/move_base", Twist, self.move_base_cb)
        self.auto_dock_sub = rospy.Subscriber("/cmd_vel/auto_dock", TwistStamped, self.auto_dock_cb)
        self.soft_estop_enable_sub = rospy.Subscriber("/soft_estop/enable", Bool, self.soft_estop_enable_cb)
        self.soft_estop_reset_sub = rospy.Subscriber("/soft_estop/reset", Bool, self.soft_estop_reset_cb)

        # ROS Publishers
        self.managed_pub = rospy.Publisher('/cmd_vel/managed', TwistStamped, queue_size=1)
        self.move_base_cancel = rospy.Publisher('/move_base/cancel', GoalID,  queue_size=1)
        self.soft_estop_enable_debounce = rospy.Publisher('/soft_estop/enable', Bool,  queue_size=1)
        self.soft_estop_reset_debounce = rospy.Publisher('/soft_estop/reset', Bool,  queue_size=1)

        self.lock_release_timer = rospy.Timer(rospy.Duration(2), self.lock_release_cb)
        self.cmd_managed_timer = rospy.Timer(rospy.Duration(0.1), self.control_input_pub)

        self.debounce_msg = Bool()
        self.debounce_msg.data = False

    def control_input_pub(self, data):
        self.managed_control_input.header.seq = self.seq
        self.managed_control_input.header.stamp = rospy.Time.now()
        if self.soft_estop:
            self.managed_control_input.twist.linear.x=0
            self.managed_control_input.twist.angular.y=0
            self.managed_control_input.twist.angular.z=0
        self.managed_pub.publish(self.managed_control_input)
        self.seq += 1

    def joystick_cb(self, joy_cmd_vel):
        if not self.remote_control_lock:
            self.managed_control_input = joy_cmd_vel
            # If a user starts to command the robot with a joystick, set local lock
            if joy_cmd_vel.twist.linear.x != 0 or joy_cmd_vel.twist.angular.y != 0 or joy_cmd_vel.twist.angular.z != 0:
                self.local_control_lock = True
             
    def move_base_cb(self, move_base_cmd_vel):
        rospy.logwarn("Move base is reporting")
        rospy.logwarn(self.local_control_lock)
        rospy.logwarn(self.remote_control_lock)
        if not self.local_control_lock:
            if not self.remote_control_lock:
                rospy.logwarn("I'm listening to move base")
                self.managed_control_input.twist = move_base_cmd_vel 

    def auto_dock_cb(self, auto_dock_cmd_vel):
        if not self.local_control_lock:
            if not self.remote_control_lock:
                self.managed_control_input = auto_dock_cmd_vel

    def soft_estop_enable_cb(self, data):
        if data.data == True:
            self.soft_estop = True
            cancel_msg=GoalID()
            self.move_base_cancel.publish(cancel_msg)
            self.soft_estop_enable_debounce.publish(self.debounce_msg)
            rospy.logwarn("Soft E-Stop Enabled")

    def soft_estop_reset_cb(self, data):
        if data.data == True:    
            self.soft_estop = False
            self.soft_estop_enable_debounce.publish(self.debounce_msg)
            rospy.logwarn("Soft E-Stop reset")

    def lock_release_cb(self, data):
        self.local_control_lock = False
        self.remote_control_lock = False


if __name__ == '__main__':
    rospy.init_node("cmdl_vel_manage_node")
    my_manager = CmdVelManager()
    rospy.spin()
