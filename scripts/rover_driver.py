#!/usr/bin/env python
import struct
import serial
import rospy
import binascii
import sys
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist

class RoverDriver:

    def __init__(self):

        battery_state_pub = rospy.Publisher('rover/battery/soc', Int32, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCB)

        try:
            self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=0.2)
        except serial.serialutil.SerialException:
            rospy.logwarn("Failed to open port")
            sys.exit(0)

    def write_to_dev(self, left_motor, right_motor, flipper, param1, param2):
        buf_out = bytearray()
        buf_out.append(0xfd)
        buf_out.append(left_motor)
        buf_out.append(right_motor)
        buf_out.append(flipper)
        buf_out.append(param1)
        buf_out.append(param2)
        buf_out.append(0x37)
        print(buf_out)
    
        try:
            self.ser.write(buf_out)
            buf_in = bytearray(self.ser.read(2))
            rospy.loginfo("Writing, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))

        except serial.SerialException:
            rospy.logerr("Failed writing to port")
            rospy.logerr(serial.SerialException)
            return False

        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            return False
        return True

    def cmdVelCB(self, msg):
        rospy.loginfo(msg)
        left_motor = int(50 + (10*msg.linear.x) + (10*msg.angular.z))
        right_motor = int(50 + (10*msg.linear.x) - (10*msg.angular.z))
        #left_motor_hex =  format(left_motor, '#04x')
        #right_motor_hex =  format(right_motor, '#04x')
        left_motor_hex = 0x35
        right_motor_hex = 0x35
        flipper = 0x00
        param1 = 0x00
        param2 = 0x00
        self.write_to_dev(left_motor_hex, right_motor_hex, flipper, param1, param2)    


if __name__ == '__main__':
    try:
        rospy.init_node('rover_driver_node')
        myRover = RoverDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Rover Driver has died")
