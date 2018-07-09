#!/usr/bin/env python

# Author: Nick Fragale
# Description: This script converts Joystick commands into Joint Velocity commands
# Four buttons: emergency stop button (button B/red), prevents further joystick 
# commands and cancels any existing move_base command
# sends zeros to cmd_vel while in ESTOP state, 
# can be toggled by pressing A button (green) after debounce time
# Monitors X and Y buttons and toggles their state (False on startup) publishes 
# a latched Bool() for X, Y and B buttons as /joystick/<x_, y_ and e_stop>_button

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
last_estop_button=time.time()
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

USE_XPAD=False
# difference between xboxdrv USB driver and xpad.ko kernel module
# xboxdrv has only 11 indices
# and the U/D_PAD_BUTTON is the 7th axis
# xpad module uses 2, xboxdrv uses 3

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

MAX_VEL = 0.25
FULL_THROTTLE = 0.1 
ADJ_THROTTLE = True
INC = 0.05 
DEADBAND = 0.2
FWD_ACC_LIM = 0.01
TRN_ACC_LIM = 0.01

prev_fwd = 0
prev_trn = 0

PREV_CMD_TIME = 0
PREV_SEQ_NUM = 0

y_button_msg = Bool()
y_button_msg.data=False
x_button_msg = Bool()
x_button_msg.data=False
e_stop_msg = Bool()
e_stop_msg.data=False


# cmd_vel publisher  
global pub
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
e_stop_pub = rospy.Publisher('/joystick/e_stop', Bool, queue_size=1, latch=True)
x_button_pub = rospy.Publisher('/joystick/x_button', Bool, queue_size=1, latch=True)
y_button_pub = rospy.Publisher('/joystick/y_button', Bool, queue_size=1, latch=True)
pub_delay = rospy.Publisher('/joystick/delay', Float32, queue_size=3)
pub_cancel_move_base = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

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
    global FULL_THROTTLE
    global MAX_VEL
    global INC
    global PREV_CMD_TIME
    global PREV_SEQ_NUM

    global cmd
    global pub_cancel_move_base
    global last_estop_button
    global last_x_button
    global last_y_button
    global last_joycb_device_check
    global e_stop_pub, e_stop_msg, x_button_pub, x_button_msg, y_button_pub, y_button_msg
    global pub

    cmd_time = float(Joy.header.stamp.secs) + (float(Joy.header.stamp.nsecs)/1000000000)
    rbt_time = time.time()
    signal_delay = rbt_time - cmd_time

    joy_delay = Float32()
    joy_delay.data = signal_delay
    pub_delay.publish(joy_delay)

    # len==8, 11 for axes,buttons with the xboxdrv
    # len==8, 15 for axes,buttons with the xpad.ko module
    if (len(Joy.axes)==8 and len(Joy.buttons)==11):
        # UP/DOWN buttons are on axes with xpad.ko
        USE_XPAD=True
        TURN_JOY=2
        U_PAD_BUTTON=7
        D_PAD_BUTTON=7
        U_PAD_BUTTON_VALUE=1
        D_PAD_BUTTON_VALUE=-1
    else:
        USE_XPAD=False
        TURN_JOY=3
        U_PAD_BUTTON = 13
        D_PAD_BUTTON = 14
        U_PAD_BUTTON_VALUE=1
        D_PAD_BUTTON_VALUE=1

    #Record timestamp and seq for use in next loop
    PREV_CMD_TIME = cmd_time
    PREV_SEQ_NUM = Joy.header.seq

    # check for e-stop button (red/B)
    if Joy.buttons[B_BUTTON] == 1:
        # debounce 1 second
        if (time.time()-last_estop_button > 1.0):
            last_estop_button=time.time()
            # go into e-stop mode (whether we're already in it or not
            e_stop_msg.data=True
            e_stop_pub.publish(e_stop_msg)
            cmd.linear.x = 0
            cmd.angular.z = 0
            rospy.loginfo('User indicated E_STOP. Canceling any move_base commands, going to E_STOP state.')
            pub.publish(cmd)
            # cancel any existing move_base command
            nogoal = GoalID()
            pub_cancel_move_base.publish(nogoal)
            return
    # check for remove-e-stop button (green/A)
    if Joy.buttons[A_BUTTON] == 1:
        # debounce 1 second
        if (time.time()-last_estop_button > 1.0):
            last_estop_button=time.time()
            # toggle e-stop mode
            if (e_stop_msg.data==1):
                e_stop_msg.data=False
                e_stop_pub.publish(e_stop_msg)
                rospy.loginfo('User indicated leaving E_STOP. Going back to regular mode.')

    # check for other two user-defined buttons. We only debounce them and monitor on/off status on a latched pub
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
    if Joy.buttons[Y_BUTTON] == 1:
        if (time.time()-last_y_button > 2.0):
            last_y_button=time.time()
            rospy.loginfo('User button Y')
            if (y_button_msg.data):
				global pub
				y_button_msg.data = False
				pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
            else:
				global pub
				y_button_msg.data = True
				pub = rospy.Publisher('/cmd_vel_temp', Twist, queue_size=3)
            y_button_pub.publish(y_button_msg)

    # stay in E_STOP mode until the button is pressed again
    if (e_stop_msg.data):
        # keep sending zeros
        cmd.linear.x = 0
        cmd.angular.z = 0
        pub.publish(cmd)
        return
    
    if ADJ_THROTTLE:
        # Increase/Decrease Max Speed
        if (USE_XPAD):
            if int(Joy.axes[U_PAD_BUTTON]) == U_PAD_BUTTON_VALUE:
                FULL_THROTTLE += INC
            if int(Joy.axes[D_PAD_BUTTON]) == D_PAD_BUTTON_VALUE:
                FULL_THROTTLE -= INC
        else:
            if Joy.buttons[U_PAD_BUTTON] == U_PAD_BUTTON_VALUE:
                FULL_THROTTLE += INC
            if Joy.buttons[D_PAD_BUTTON] == D_PAD_BUTTON_VALUE:
                FULL_THROTTLE -= INC
        
        # If the user tries to decrese full throttle to 0
        # Then set it back up to 0.2 m/s
        if FULL_THROTTLE <= 0.001:
            FULL_THROTTLE = INC

        # If the user tries to INCreas the velocity limit when its at max
        # then set velocity limit to max allowed velocity
        if FULL_THROTTLE >= MAX_VEL:
            FULL_THROTTLE = MAX_VEL

        # Report the new FULL_THROTTLE
        #if Joy.buttons[D_PAD_BUTTON] or Joy.buttons[U_PAD_BUTTON]:
        #    rospy.loginfo(FULL_THROTTLE)

        #Update DEADBAND
        DEADBAND = 0.2 * FULL_THROTTLE

    # Drive Forward/Backward commands
    drive_cmd = FULL_THROTTLE * Joy.axes[DRIVE_JOY] #left joystick
    if drive_cmd < DEADBAND and -DEADBAND < drive_cmd:
        drive_cmd = 0 
        
    # Turn left/right commands
    turn_cmd = FULL_THROTTLE * Joy.axes[TURN_JOY] #right joystick
    if turn_cmd < DEADBAND and -DEADBAND < turn_cmd:
        turn_cmd = 0

    # Flipper up/down commands
    flipper_cmd = (FULL_THROTTLE * Joy.axes[L_TRIG_AXES]) - (FULL_THROTTLE * Joy.axes[R_TRIG_AXES])
    if flipper_cmd < DEADBAND and -DEADBAND < flipper_cmd:
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
    e_stop_pub.publish(e_stop_msg)
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

