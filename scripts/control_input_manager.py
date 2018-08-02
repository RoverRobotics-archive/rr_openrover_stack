#!/usr/bin/env python

# Author: Nick Fragale
# Description: This script manages cmd_vel from multiple sources so that they don't over-ride eachother, and so that soft E-stop can works from multiple sources. 


import rospy
import time
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class CmdVelManager(object):

    use_joystick = True
    soft_estop = False
    local_control_lock = False
    remote_control_lock = False

    def __init__(self):

        self.joystick_sub = rospy.Subscriber("/cmd_vel/joystick", Twist, self.joystick_cb)
        self.move_base_sub = rospy.Subscriber("/cmd_vel/move_base", Twist, self.move_base_cb)
        self.auto_dock_sub = rospy.Subscriber("/cmd_vel/auto_dock", Twist, self.auto_dock_cb)
        self.auto_dock_sub = rospy.Subscriber("/soft_estop/enable", Bool, self.soft_estop_enable_cb)
        self.auto_dock_sub = rospy.Subscriber("/soft_estop/reset", Bool, self.soft_estop_reset_cb)

        # ROS Publishers
        self.managed_pub = rospy.Publisher('/cmd_vel/managed', Twist, queue_size=1)
        self.move_base_cancel = rospy.Publisher('/move_base/cancel', GoalID,  queue_size=1)
        self.soft_estop_enable_debounce = rospy.Publisher('/soft_estop/enable', Bool,  queue_size=1)
        self.soft_estop_reset_debounce = rospy.Publisher('/soft_estop/reset', Bool,  queue_size=1)

        # Get parameters values
        self.remote_timeout = rospy.get_param('~remote_timeout', '0.5')
        self.local_timeout = rospy.get_param('~local_timeout', '0.5')
        self.max_vel = rospy.get_param('~max_vel', '0.5')

        self.lock_release_timer = rospy.Timer(rospy.Duration(2), self.lock_release_cb)

        self.debounce_msg = Bool()
        self.debounce_msg.data = False

    # Joystick Callback 
    def joystick_cb(self, joy_cmd_vel):
        if not self.soft_estop:
            if not self.remote_control_lock:
                self.local_control_lock = True
                self.managed_pub.publish(joy_cmd_vel)
             
    # Move Base Callback
    def move_base_cb(self, move_base_cmd_vel):
        if not self.soft_estop:
            if not self.local_control_lock:
                if not self.remote_control_lock:
                    self.managed_pub.publish(move_base_cmd_vel)

    # Auto Dock Callback
    def auto_dock_cb(self, joy_cmd_vel):
        if not self.soft_estop:
            if not self.local_control_lock:
                if not self.remote_control_lock:
                    self.managed_pub.publish(managed_cmd_vel)

    # Estop Callbacks
    def soft_estop_enable_cb(self, data):
        if data == True:
            self.soft_estop = True
            cancel_msg=GoalID()
            self.move_base_cancel.Publish(cancel_msg)
            self.soft_estop_enable_debounce.Publish(self.debounce_msg)
    def soft_estop_reset_cb(self, data):
        if data == True:    
            self.soft_estop = False
            self.soft_estop_enable_debounce.Publish(self.debounce_msg)

    def lock_release_cb(self):
        local_control_lock = False
        remote_control_lock = False


if __name__ == '__main__':
    rospy.init_node("cmdl_vel_manage_node")
    r = rospy.Rate(10) # 10hz
    my_manager = CmdVelManager()
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()
