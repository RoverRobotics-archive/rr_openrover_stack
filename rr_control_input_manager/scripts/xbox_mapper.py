#!/usr/bin/env python
from __future__ import division

# Author: Nick Fragale
# Description: This script converts Joystick commands into Joint Velocity commands
# Monitors A, B, X and Y buttons and toggles their state (False on startup) publishes
# a latched Bool() to /joystick/<button> where button is A, B, Y, or X
# these can be remapped to different topics to control various things like E-stoping the robot
# or starting to record a bagfile, or taking a picture.

# Xbox controller mapping:
#   axes: [l-stick horz,l-stick vert, l-trigger, r-stick horz, r-stick vert, r-trigger]
#   buttons: [a,b,x,y,lb,rb,back,start,xbox,l-stick,r-stick,l-pad,r-pad,u-pad,d-pad]

import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float32, String
import rosnode

rospy.init_node('xbox_mapper_node', anonymous=True)

cmd = TwistStamped()
seq = 0
last_a_button = time.time()
last_b_button = time.time()
last_x_button = time.time()
last_y_button = time.time()
last_joycb_device_check = time.time()
button_msg = String()

# difference between xboxdrv USB driver and xpad.ko kernel module
# 1) xboxdrv has only 11 indices
# 2) U/D_PAD_BUTTON is the 7th axis
# 3) xpad module uses 2, xboxdrv uses 3
driver = rospy.get_param('/xbox_mapper_node/driver', 'xboxdrv')
wired_or_wireless = rospy.get_param('/xbox_mapper_node/wired_or_wireless', 'wireless')

if driver == 'xpad':
    rospy.logfatal('[{node_name}] xpad driver not supported.'.format(node_name=rospy.get_name()))
    rospy.signal_shutdown('xpad driver not supported.')
    exit(-1)

elif wired_or_wireless == 'wired' and driver == 'xboxdrv':
    rospy.loginfo('XBOX CONFIG: wired & xboxdrv')
    rospy.logwarn('If the wired controller becomes unplugged during operation '
                  'xboxdrv may continue to publish the last command from the '
                  'controller, causing the vehicle to run away.')

elif wired_or_wireless == 'wireless' and driver == 'xboxdrv':
    rospy.loginfo('XBOX CONFIG: wireless & xboxdrv')

else:
    rospy.logfatal('Unsupported controller configuration: {driver}, {connection}'
                   .format(driver=driver, connection=wired_or_wireless))
    rospy.signal_shutdown('Unsupported controller configuration.')
    exit(-1)

L_STICK_H_AXES = 0
L_STICK_V_AXES = 1
L_TRIG_AXES = 5
R_STICK_H_AXES = 2
R_STICK_V_AXES = 3
R_TRIG_AXES = 4
DPAD_H_AXES = 6
DPAD_V_AXES = 7

A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3
LB_BUTTON = 4
RB_BUTTON = 5
BACK_BUTTON = 6
START_BUTTON = 7
POWER_BUTTON = 8
L_STICK_BUTTON = 9
R_STICK_BUTTON = 10

prev_fwd = 0
prev_trn = 0

PREV_CMD_TIME = 0
PREV_SEQ_NUM = 0

MAX_VEL_FWD = rospy.get_param('~max_vel_drive', 2.6)
MAX_VEL_TURN = rospy.get_param('~max_vel_turn', 9.0)
MAX_VEL_FLIPPER = rospy.get_param('~max_vel_flipper', 1.4)
DRIVE_THROTTLE = rospy.get_param('~default_drive_throttle', 0.15)
FLIPPER_THROTTLE = rospy.get_param('~default_flipper_throttle', 0.6)
ADJ_THROTTLE = rospy.get_param('~adjustable_throttle', True)
A_BUTTON_TOGGLE = rospy.get_param('~a_button_toggle', False)
B_BUTTON_TOGGLE = rospy.get_param('~b_button_toggle', False)
X_BUTTON_TOGGLE = rospy.get_param('~x_button_toggle', False)
Y_BUTTON_TOGGLE = rospy.get_param('~y_button_toggle', False)
MIN_TOGGLE_DUR = 0.5
DRIVE_INCREMENTS = rospy.get_param('~drive_increment', 20.0)
FLIPPER_INCREMENTS = rospy.get_param('~drive_increment', 20.0)
DEADBAND = 0.2
FWD_ACC_LIM = 0.2
TRN_ACC_LIM = 0.4
DPAD_ACTIVE = False

a_button_msg = Bool()
a_button_msg.data = False
b_button_msg = Bool()
b_button_msg.data = False
x_button_msg = Bool()
x_button_msg.data = False
y_button_msg = Bool()
y_button_msg.data = False

# define publishers  
pub = rospy.Publisher('/cmd_vel/joystick', TwistStamped, queue_size=3)
a_button_pub = rospy.Publisher('/joystick/a_button', Bool, queue_size=1, latch=True)
b_button_pub = rospy.Publisher('/joystick/b_button', Bool, queue_size=1, latch=True)
x_button_pub = rospy.Publisher('/joystick/x_button', Bool, queue_size=1, latch=True)
y_button_pub = rospy.Publisher('/joystick/y_button', Bool, queue_size=1, latch=True)
pub_delay = rospy.Publisher('/joystick/delay', Float32, queue_size=3)
pub_cancel_move_base = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)


def limit_acc(fwd, trn):
    # TODO calculate delta t and times acc limits by that so that acc_limits are correct units
    global prev_fwd, prev_trn

    fwd_acc = fwd - prev_fwd
    if fwd_acc > FWD_ACC_LIM:
        fwd = prev_fwd + FWD_ACC_LIM
    elif fwd_acc < -FWD_ACC_LIM:
        fwd = prev_fwd - FWD_ACC_LIM

    trn_acc = trn - prev_trn
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
    global DPAD_ACTIVE

    global cmd
    global seq
    global last_a_button, last_b_button, last_x_button, last_y_button
    global last_joycb_device_check
    global a_button_pub, a_button_msg, b_button_pub, b_button_msg
    global x_button_pub, x_button_msg, y_button_pub, y_button_msg

    cmd_time = float(Joy.header.stamp.secs) + (float(Joy.header.stamp.nsecs) / 1000000000)
    rbt_time = time.time()
    signal_delay = rbt_time - cmd_time

    joy_delay = Float32()
    joy_delay.data = signal_delay
    pub_delay.publish(joy_delay)

    # Record timestamp and seq for use in next loop
    PREV_CMD_TIME = cmd_time
    PREV_SEQ_NUM = Joy.header.seq

    # check for other two user-defined buttons. We only debounce them and monitor on/off status on a latched pub
    # (green/A)
    if A_BUTTON_TOGGLE:
        if Joy.buttons[A_BUTTON] == 1:
            if time.time() - last_a_button > MIN_TOGGLE_DUR:
                last_a_button = time.time()
                a_button_state = not a_button_msg.data
                rospy.loginfo('A button toggled: {state}'.format(state=a_button_state))
                a_button_msg.data = a_button_state
    else:
        if Joy.buttons[A_BUTTON] == 1:
            a_button_msg.data = True
        else:
            a_button_msg.data = False
    a_button_pub.publish(a_button_msg)

    # (red/B)
    if B_BUTTON_TOGGLE:
        if Joy.buttons[B_BUTTON] == 1:
            if time.time() - last_b_button > MIN_TOGGLE_DUR:
                last_b_button = time.time()
                b_button_state = not b_button_msg.data
                rospy.loginfo('B button toggled: {state}'.format(state=b_button_state))
                b_button_msg.data = b_button_state
    else:
        if Joy.buttons[B_BUTTON] == 1:
            b_button_msg.data = True
        else:
            b_button_msg.data = False
    b_button_pub.publish(b_button_msg)

    # (blue/X)
    if X_BUTTON_TOGGLE:
        if Joy.buttons[X_BUTTON] == 1:
            if time.time() - last_x_button > MIN_TOGGLE_DUR:
                last_x_button = time.time()
                x_button_state = not x_button_msg.data
                rospy.loginfo('X button toggled: {state}'.format(state=x_button_state))
                x_button_msg.data = x_button_state
    else:
        if Joy.buttons[X_BUTTON] == 1:
            x_button_msg.data = True
        else:
            x_button_msg.data = False
    x_button_pub.publish(x_button_msg)

    # (yellow/Y)
    if Y_BUTTON_TOGGLE:
        if Joy.buttons[Y_BUTTON] == 1:
            if time.time() - last_y_button > MIN_TOGGLE_DUR:
                last_y_button = time.time()
                y_button_state = not y_button_msg.data
                rospy.loginfo('Y button toggled: {state}'.format(state=y_button_state))
                y_button_msg.data = y_button_state
    else:
        if Joy.buttons[Y_BUTTON] == 1:
            y_button_msg.data = True
        else:
            y_button_msg.data = False
    y_button_pub.publish(y_button_msg)

    if ADJ_THROTTLE:
        # Increase/Decrease Max Speed
        if (driver == 'xpad'):
            if int(Joy.axes[DPAD_V_AXES]) == 1 and not DPAD_ACTIVE:
                DRIVE_THROTTLE += (1 / DRIVE_INCREMENTS)
                DPAD_ACTIVE = True
            if int(Joy.axes[DPAD_V_AXES]) == -1 and not DPAD_ACTIVE:
                DRIVE_THROTTLE -= (1 / DRIVE_INCREMENTS)
                DPAD_ACTIVE = True
        elif (driver == 'xboxdrv'):
            if int(Joy.axes[DPAD_V_AXES]) == 1 and not DPAD_ACTIVE:
                DRIVE_THROTTLE += (1 / DRIVE_INCREMENTS)
                DPAD_ACTIVE = True
            if int(Joy.axes[DPAD_V_AXES]) == -1 and not DPAD_ACTIVE:
                DRIVE_THROTTLE -= (1 / DRIVE_INCREMENTS)
                DPAD_ACTIVE = True

        if Joy.buttons[LB_BUTTON] == 1:
            FLIPPER_THROTTLE -= (1 / FLIPPER_INCREMENTS)
            rospy.loginfo(FLIPPER_THROTTLE)
        if Joy.buttons[RB_BUTTON] == 1:
            FLIPPER_THROTTLE += (1 / FLIPPER_INCREMENTS)
            rospy.loginfo(FLIPPER_THROTTLE)

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

        # Update DEADBAND
        FWD_DEADBAND = 0.2 * DRIVE_THROTTLE * MAX_VEL_FWD
        TURN_DEADBAND = 0.2 * DRIVE_THROTTLE * MAX_VEL_TURN
        FLIPPER_DEADBAND = 0.2 * FLIPPER_THROTTLE * MAX_VEL_FLIPPER

        if DPAD_ACTIVE:
            rospy.loginfo('Drive Throttle: %f', DRIVE_THROTTLE)

        if (Joy.axes[DPAD_V_AXES], Joy.axes[DPAD_H_AXES]) == (0, 0):
            DPAD_ACTIVE = False

    # Drive Forward/Backward commands
    drive_cmd = DRIVE_THROTTLE * MAX_VEL_FWD * Joy.axes[L_STICK_V_AXES]  # left joystick
    if drive_cmd < FWD_DEADBAND and -FWD_DEADBAND < drive_cmd:
        drive_cmd = 0

        # Turn left/right commands
    turn_cmd = (1.1 - (drive_cmd / MAX_VEL_FWD)) * DRIVE_THROTTLE * MAX_VEL_TURN * Joy.axes[
        R_STICK_H_AXES]  # right joystick
    if turn_cmd < TURN_DEADBAND and -TURN_DEADBAND < turn_cmd:
        turn_cmd = 0

    # Flipper up/down commands
    flipper_cmd = (FLIPPER_THROTTLE * MAX_VEL_FLIPPER * Joy.axes[L_TRIG_AXES]) - (
            FLIPPER_THROTTLE * MAX_VEL_FLIPPER * Joy.axes[R_TRIG_AXES])
    if flipper_cmd < FLIPPER_DEADBAND and -FLIPPER_DEADBAND < flipper_cmd:
        flipper_cmd = 0

    # Limit acceleration
    # drive_cmd, turn_cmd = limit_acc(drive_cmd, turn_cmd)

    # update the last time joy_cb was called
    if (drive_cmd != 0) or (turn_cmd != 0):
        last_joycb_device_check = time.time()

    # Publish move commands
    cmd.header.seq = seq
    cmd.header.stamp = rospy.Time.now()
    cmd.twist.linear.x = drive_cmd
    cmd.twist.angular.y = flipper_cmd
    cmd.twist.angular.z = turn_cmd
    pub.publish(cmd)
    seq += 1


# Main Function
def joystick_main():
    # Initialize driver node
    r = rospy.Rate(10)  # 10hz
    # publish the latched button initializations
    a_button_pub.publish(a_button_msg)
    b_button_pub.publish(b_button_msg)
    x_button_pub.publish(x_button_msg)
    y_button_pub.publish(y_button_msg)

    while not rospy.is_shutdown():
        # Subscribe to the joystick topic
        sub_cmds = rospy.Subscriber('joystick', Joy, joy_cb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        r.sleep()


if __name__ == '__main__':
    try:
        joystick_main()
    except rospy.ROSInterruptException:
        pass
