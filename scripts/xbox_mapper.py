#!/usr/bin/env python

# Author: Nick Fragale
# Description: This script converts Joystick commands into Joint Velocity commands
# Monitors A, B, X and Y buttons and toggles their state (False on startup) publishes 
# a latched Bool() to /joystick/<button> where button is A, B, Y, or X
# these can be remapped to different topics to control various things like E-stoping the robot
# or starting to record a bagfile, or taking a picture.

# Xbox controller mapping:
#   axes: [l-stick horz,l-stick vert, l-trigger, r-stick horz, r-stick vert, r-trigger]
#   buttons: [a,b,x,y,lb,rb,back,start,xbox,l-stick,r-stick,l-pad,r-pad,u-pad,d-pad]  

import numpy as np
import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
import os
import time
from std_msgs.msg import Bool

cmd = Twist()
last_a_button=time.time()
last_b_button=time.time()
last_x_button=time.time()
last_y_button=time.time()
last_joycb_device_check=time.time()
button_msg = String()

L_STICK_H_AXES = 0
L_STICK_V_AXES = 1
L_TRIG_AXES = 2
R_STICK_H_AXES = 3 
R_STICK_V_AXES = 4
R_TRIG_AXES = 5

DRIVE_JOY=1
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3
LB_BUTTON = 4
RB_BUTTON = 5
BACK_BUTTON = 6
START_BUTTON = 7
L_STICK_BUTTON = 8
R_STICK_BUTTON = 9
L_PAD_BUTTON = 11
R_PAD_BUTTON = 12
U_PAD_BUTTON = 13
D_PAD_BUTTON = 14

prev_fwd = 0
prev_trn = 0

PREV_CMD_TIME = 0
PREV_SEQ_NUM = 0



# difference between xboxdrv USB driver and xpad.ko kernel module
# 1) xboxdrv has only 11 indices
# 2) U/D_PAD_BUTTON is the 7th axis
# 3) xpad module uses 2, xboxdrv uses 3
driver = rospy.get_param('~driver', 'xpad')
wired_or_wireless = rospy.get_param('~wired_or_wireless', 'wireless')

MAX_VEL_FWD = rospy.get_param('~max_vel_drive', 2.6)
MAX_VEL_TURN = rospy.get_param('~max_vel_turn', 1.4)
MAX_VEL_FLIPPER = rospy.get_param('~max_vel_flipper', 1.4)
DRIVE_THROTTLE = rospy.get_param('~default_drive_throttle', 0.6)
FLIPPER_THROTTLE = rospy.get_param('~default_flipper_throttle', 0.6)
ADJ_THROTTLE = rospy.get_param('~adjustable_throttle', True)
DRIVE_INCREMENTS = 10
FLIPPER_INCREMENTS = 10
DEADBAND = 0.2
FWD_ACC_LIM = 0.01
TRN_ACC_LIM = 0.01

a_button_msg = Bool()
a_button_msg.data=False
b_button_msg = Bool()
b_button_msg.data=False
x_button_msg = Bool()
x_button_msg.data=False
y_button_msg = Bool()
y_button_msg.data=False

# define publishers  
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
a_button_pub = rospy.Publisher('/joystick/a_button', Bool, queue_size=1, latch=True)
b_button_pub = rospy.Publisher('/joystick/b_button', Bool, queue_size=1, latch=True)
x_button_pub = rospy.Publisher('/joystick/x_button', Bool, queue_size=1, latch=True)
y_button_pub = rospy.Publisher('/joystick/y_button', Bool, queue_size=1, latch=True)
pub_delay = rospy.Publisher('/joystick/delay', Float32, queue_size=3)
pub_cancel_move_base = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

# Function deprecated, accel limiting is done in firmware now.
def limit_acc(fwd,trn):

    fwd_acc = fwd - prev_fwd
    if fwd_acc > FWD_ACC_LIM:
        fwd = prev_fwd + FWD_ACC_LIM
    elif fwd_acc < -FWD_ACC_LIM:
        fwd = prev_fwd - FWD_ACC_LIM

    turn_acc = trn - prev_trn
    if trn_acc > TRN_ACC_LIM:
        trn = prev_trn + TRN_ACC_LIM
    elif trn_acc < -TRN_ACC_LIM:
        trn = prev_trn - TRN_ACC_LIM
    
    prev_fwd = fwd
    prev_trn = trn

    return fwd, trn


def joy_cb(Joy):
    global ADJ_THROTTLE
    global MAX_VEL_FWD
    global MAX_VEL_TURN
    global MAX_VEL_FLIPPER
    global DRIVE_THROTTLE
    global FLIPPER_THROTTLE
    global DRIVE_INCREMENTS
    global FLIPPER_INCREMENTS
    global PREV_CMD_TIME
    global PREV_SEQ_NUM

    global cmd
    global last_a_button, last_b_button, last_x_button, last_y_button
    global last_joycb_device_check
    global a_button_pub, a_button_msg, b_button_pub, b_button_msg
    global x_button_pub, x_button_msg, y_button_pub, y_button_msg
    

    cmd_time = float(Joy.header.stamp.secs) + (float(Joy.header.stamp.nsecs)/1000000000)
    rbt_time = time.time()
    signal_delay = rbt_time - cmd_time

    joy_delay = Float32()
    joy_delay.data = signal_delay
    pub_delay.publish(joy_delay)

    # len==8, 11 for axes,buttons with the xboxdrv
    # len==8, 15 for axes,buttons with the xpad.ko module
    if (driver == "xpad"):
        # UP/DOWN buttons are on axes with xpad.ko
        TURN_JOY=2
        U_PAD_BUTTON=7
        D_PAD_BUTTON=7
    elif (driver == "xboxdrv"):
        TURN_JOY=3
        U_PAD_BUTTON = 13
        D_PAD_BUTTON = 14

    #Record timestamp and seq for use in next loop
    PREV_CMD_TIME = cmd_time
    PREV_SEQ_NUM = Joy.header.seq


    # check for other two user-defined buttons. We only debounce them and monitor on/off status on a latched pub
    # (green/A)
    if Joy.buttons[A_BUTTON] == 1:
        if (time.time()-last_a_button > 2.0):
            last_a_button=time.time()
            rospy.loginfo('User button A')
            # toggle button
            if (a_button_msg.data):
                a_button_msg.data = False
            else:
                a_button_msg.data = True
            a_button_pub.publish(a_button_msg)

    # (red/B)
    if Joy.buttons[B_BUTTON] == 1:
        if (time.time()-last_b_button > 2.0):
            last_b_button=time.time()
            rospy.loginfo('User button b')
            # toggle button
            if (b_button_msg.data):
                b_button_msg.data = False
            else:
                b_button_msg.data = True
            b_button_pub.publish(b_button_msg)

    # (blue/X)
    if Joy.buttons[X_BUTTON] == 1:
        if (time.time()-last_x_button > 2.0):
            last_x_button=time.time()
            rospy.loginfo('User button X')
            # toggle button
            if (x_button_msg.data):
                x_button_msg.data = False
            else:
                x_button_msg.data = True
            x_button_pub.publish(x_button_msg)

    # (yellow/Y)
    if Joy.buttons[Y_BUTTON] == 1:
        if (time.time()-last_y_button > 2.0):
            last_y_button=time.time()
            rospy.loginfo('User button Y')
            if (y_button_msg.data):
                y_button_msg.data = False
            else:
                y_button_msg.data = True
            y_button_pub.publish(y_button_msg)
    
    if ADJ_THROTTLE:
        # Increase/Decrease Max Speed
        if (driver == "xpad"):
            if int(Joy.axes[U_PAD_BUTTON]) == 1:
                DRIVE_THROTTLE += (1 / DRIVE_INCREMENTS)
            if int(Joy.axes[D_PAD_BUTTON]) == -1:
                DRIVE_THROTTLE -= (1 / DRIVE_INCREMENTS) 
        elif (driver == "xboxdrv"):
            if Joy.buttons[U_PAD_BUTTON] == 1:
                DRIVE_THROTTLE += (1 / DRIVE_INCREMENTS)
            if Joy.buttons[D_PAD_BUTTON] == 1:
                DRIVE_THROTTLE -= (1 / DRIVE_INCREMENTS)

        if Joy.buttons[LB_BUTTON] == 1:
                FLIPPER_THROTTLE += (1 / FLIPPER_INCREMENTS)
        if Joy.buttons[RB_BUTTON] == 1:
                FLIPPER_THROTTLE -= (1 / FLIPPER_INCREMENTS)
        
        # If the user tries to decrease full throttle to 0
        # Then set it back up to 0.2 m/s
        if DRIVE_THROTTLE <= 0.001:
            DRIVE_THROTTLE = (1 / DRIVE_INCREMENTS)
        if FLIPPER_THROTTLE <= 0.001:
            FLIPPER_THROTTLE = (1 / FLIPPER_INCREMENTS)

        # If the user tries to increase the velocity limit when its at max
        # then set velocity limit to max allowed velocity
        if DRIVE_THROTTLE >= 1:
            DRIVE_THROTTLE = 1
        if FLIPPER_THROTTLE >= 1:
            FLIPPER_THROTTLE = 1


        #Update DEADBAND
        FWD_DEADBAND = 0.2 * DRIVE_THROTTLE * MAX_VEL_FWD
        TURN_DEADBAND = 0.2 * DRIVE_THROTTLE * MAX_VEL_TURN
        FLIPPER_DEADBAND = 0.2 * FLIPPER_THROTTLE * MAX_VEL_FLIPPER

    # Drive Forward/Backward commands
    drive_cmd = FWD_THROTTLE * MAX_VEL_FWD * Joy.axes[DRIVE_JOY] #left joystick
    if drive_cmd < DRIVE_DEADBAND and -DRIVE_DEADBAND < drive_cmd:
        drive_cmd = 0 
        
    # Turn left/right commands
    turn_cmd = TURN_THROTTLE * MAX_VEL_TURN * Joy.axes[TURN_JOY] #right joystick
    if turn_cmd < DRIVE_DEADBAND and -DRIVE_DEADBAND < turn_cmd:
        turn_cmd = 0

    # Flipper up/down commands
    flipper_cmd = (FLIPPER_THROTTLE * MAX_VEL_FLIPPER * Joy.axes[L_TRIG_AXES]) - (FLIPPER_THROTTLE * MAX_VEL_FLIPPER * Joy.axes[R_TRIG_AXES])
    if flipper_cmd < FLIPPER_DEADBAND and -FLIPPER_DEADBAND < flipper_cmd:
        flipper_cmd = 0

    #Limit acceleration
    #drive_cmd, turn_cmd = limit_acc(drive_cmd, turn_cmd)

    # update the last time joy_cb was called
    if (drive_cmd != 0) or (turn_cmd != 0):
        last_joycb_device_check=time.time()

    # Publish move commands
    cmd.linear.x = drive_cmd
    cmd.angular.y = flipper_cmd
    cmd.angular.z = turn_cmd
    pub.publish(cmd)
    
# Main Function
def joystick_main():

    # Initialize driver node
    rospy.init_node('joystick_node', anonymous=True)
    r = rospy.Rate(10) # 10hz
    # publish the latched button initializations
    a_button_pub.publish(a_button_msg)
    b_button_pub.publish(b_button_msg)
    x_button_pub.publish(x_button_msg)
    y_button_pub.publish(y_button_msg)

    while not rospy.is_shutdown():
        # Subscribe to the joystick topic
        sub_cmds = rospy.Subscriber("joystick", Joy, joy_cb)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        r.sleep()
    
if __name__ == '__main__':
    try:
        joystick_main()
    except rospy.ROSInterruptException:
        pass

