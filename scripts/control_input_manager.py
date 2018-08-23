#!/usr/bin/env python

# Author: Nick Fragale
# Description: This script manages cmd_vel from multiple sources so that they don't over-ride eachother, and so that soft E-stop can works from multiple sources. 


import rospy
import time
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from actionlib_msgs.msg import GoalID

class CmdVelManager(object):

    use_joystick = True
    soft_estop = False
    local_control_lock = False
    remote_control_lock = False
    command_timeout = 0.5
    move_base_control_input_request = Twist()
    auto_dock_control_input_request = TwistStamped()
    fleet_manager_control_input_request = Twist()
    joy_control_input_request = TwistStamped()
    managed_control_input = TwistStamped()
    seq = 0
    def __init__(self):

        self.joystick_sub = rospy.Subscriber("/cmd_vel/joystick", TwistStamped, self.joystick_cb)
        self.move_base_sub = rospy.Subscriber("/cmd_vel/move_base", Twist, self.move_base_cb)
        self.fleet_manager_sub = rospy.Subscriber("/cmd_vel/fleet_manager", Twist, self.fleet_manager_cb)
        self.auto_dock_sub = rospy.Subscriber("/cmd_vel/auto_dock", TwistStamped, self.auto_dock_cb)
        self.soft_estop_enable_sub = rospy.Subscriber("/soft_estop/enable", Bool, self.soft_estop_enable_cb)
        self.soft_estop_reset_sub = rospy.Subscriber("/soft_estop/reset", Bool, self.soft_estop_reset_cb)

        # ROS Publishers
        self.managed_pub = rospy.Publisher('/cmd_vel/managed', TwistStamped, queue_size=1)
        self.move_base_cancel = rospy.Publisher('/move_base/cancel', GoalID,  queue_size=1)
        self.active_controller_pub = rospy.Publisher('/active_controller', String, queue_size=1)
        self.auto_dock_cancel = rospy.Publisher('/auto_dock/cancel', Bool, queue_size = 10)

        self.cmd_managed_timer = rospy.Timer(rospy.Duration(0.1), self.control_input_pub)

        self.last_move_base_command_time = rospy.Time.now()
        self.last_auto_dock_command_time = rospy.Time.now()
        self.last_fleet_manager_command_time = rospy.Time.now()
        self.last_joy_command_time = rospy.Time.now()

    def control_input_pub(self, data):
        self.managed_control_input.header.seq = self.seq
        self.managed_control_input.header.stamp = rospy.Time.now()
        self.managed_control_input.twist.linear.x=0
        self.managed_control_input.twist.angular.y=0
        self.managed_control_input.twist.angular.z=0
        current_time = rospy.Time.now()

        #rospy.logwarn(self.local_control_lock)
        #rospy.logwarn(self.remote_control_lock)
        active_controller = String()
        active_controller.data = 'None'
        
        move_base_time_elapsed = current_time - self.last_move_base_command_time
        auto_dock_time_elapsed = current_time - self.last_auto_dock_command_time
        fleet_manager_time_elapsed = current_time - self.last_fleet_manager_command_time
        joy_time_elapsed = current_time - self.last_joy_command_time

        if joy_time_elapsed.to_sec > 2:
            self.lock_release_cb()

        # Process move_base requests
        if not self.local_control_lock:
            if not self.remote_control_lock:
                if move_base_time_elapsed.to_sec() < self.command_timeout:
                    active_controller.data = 'move_base'
                    self.managed_control_input.twist = self.move_base_control_input_request


        # Process auto_dock requests
        if not self.local_control_lock:
            if not self.remote_control_lock:
                if auto_dock_time_elapsed.to_sec() < self.command_timeout:
                    active_controller.data = 'auto_dock'
                    self.managed_control_input = self.auto_dock_control_input_request


        # Process fleet_manager requests
        if not self.local_control_lock:
            if fleet_manager_time_elapsed.to_sec() < self.command_timeout:
                active_controller.data = 'fleet_manager'
                self.managed_control_input.twist = self.fleet_manager_control_input_request
 

        # Process joystick requests
        #rospy.logwarn(joy_time_elapsed.to_sec())
        #rospy.logwarn(self.command_timeout)
        if joy_time_elapsed.to_sec() < self.command_timeout:
            active_controller.data = 'joystick'
            self.managed_control_input = self.joy_control_input_request

        #rospy.logwarn("move_base: %f", move_base_time_elapsed.to_sec())
        #rospy.logwarn("auto_dock: %f", auto_dock_time_elapsed.to_sec())
        #rospy.logwarn("fleet_manager: %f", fleet_manager_time_elapsed.to_sec())
        #rospy.logwarn("joy: %f", joy_time_elapsed.to_sec())


        # Check for estop 
        if self.soft_estop:
            self.managed_control_input.twist.linear.x=0
            self.managed_control_input.twist.angular.y=0
            self.managed_control_input.twist.angular.z=0
            rospy.logwarn_throttle(60, "[CONTROL_INPUT_MANAGER_NODE] Soft Estop is still enabled which will prevent any motion")
        self.managed_pub.publish(self.managed_control_input)
        self.active_controller_pub.publish(active_controller)

        self.seq += 1

         
    def move_base_cb(self, move_base_cmd_vel):
        #rospy.logwarn(move_base_cmd_vel)
        self.last_move_base_command_time = rospy.Time.now()
        self.move_base_control_input_request = move_base_cmd_vel 


    def auto_dock_cb(self, auto_dock_cmd_vel):
        self.last_auto_dock_command_time = rospy.Time.now()
        self.auto_dock_control_input_request = auto_dock_cmd_vel


    def fleet_manager_cb(self, fleet_manager_cmd_vel):
        self.last_fleet_manager_command_time = rospy.Time.now()
        if fleet_manager_cmd_vel.linear.x != 0 or fleet_manager_cmd_vel.angular.y != 0 or fleet_manager_cmd_vel.angular.z != 0:
            self.remote_control_lock = True
        self.fleet_manager_control_input_request = fleet_manager_cmd_vel


    def joystick_cb(self, joy_cmd_vel):
        # If a user starts to command the robot with a joystick, set local lock
        if joy_cmd_vel.twist.linear.x != 0 or joy_cmd_vel.twist.angular.y != 0 or joy_cmd_vel.twist.angular.z != 0:    
            self.last_joy_command_time = joy_cmd_vel.header.stamp
            self.local_control_lock = True
        self.joy_control_input_request = joy_cmd_vel


    def soft_estop_enable_cb(self, data):
        if data.data == True:
            self.soft_estop = True
            cancel_msg=GoalID()
            self.move_base_cancel.publish(cancel_msg)
            stop_msg = Bool()
            stop_msg.data = True
            self.auto_dock_cancel.publish(stop_msg)
            rospy.logwarn("[CONTROL_INPUT_MANAGER_NODE] Soft E-Stop Enabled")

    def soft_estop_reset_cb(self, data):
        if data.data == True:    
            self.soft_estop = False
            rospy.logwarn("[CONTROL_INPUT_MANAGER_NODE] Soft E-Stop reset")

    def lock_release_cb(self):
        self.local_control_lock = False
        self.remote_control_lock = False


if __name__ == '__main__':
    rospy.init_node("control_input_manager_node")
    my_manager = CmdVelManager()
    rospy.spin()
