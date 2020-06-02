#!/usr/bin/env python
from roboclaw_driver.roboclaw import Roboclaw
import rospy
from geometry_msgs.msg import Twist, Quaternion
from threading import Lock
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import math
from std_msgs.msg import Bool

class RoverZeroNode:
    def __init__(self):
        self._node = rospy.init_node('Rover_Zero_Controller', anonymous=True)

        # Class Variables
        self._left_effort = 64
        self._right_effort = 64
        self._serial_lock = Lock()
        self._global_var_lock = Lock()
        self._safety_lock = Lock()
        self._last_cmd_vel_received = rospy.get_rostime()
        self.e_stop_on_ = False

        # Diagnostic Parameters
        self._firmware_version = None
        self._left_motor_current = None
        self._right_motor_current = None
        self._battery_voltage = None

        # ROS params
        self._port = rospy.get_param('~dev', '/dev/ttyACM0')
        self._address = rospy.get_param('~address', 0x80)
        self._baud = rospy.get_param('~baud', 115200)
        self._timeout = rospy.get_param('~timeout', 0.1)
        self._attempts = rospy.get_param('~attempts', 1)
        self._max_vel = rospy.get_param('~max_vel', 5.0)
        self._effort_coef = rospy.get_param('~effort_coef', 0.09)
        self._max_turn = rospy.get_param('~max_turn', 6.28)
        self._turn_coeff = rospy.get_param('~turn_coefficient', 1.5)
        self._linear_coeff = rospy.get_param('~linear_coefficient', 3.0)
        self._diag_frequency = rospy.get_param('~diag_frequency', 1.0)
        self._odom_frequency = rospy.get_param('~odom_frequency', 1.0/30.0)
        self._cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout', 0.5)
        self._odom_frame = rospy.get_param('~odom_frame', "odom")
        self._base_link_frame = rospy.get_param('~base_link_frame', "base_link")
        self._wheel_base = 0.358775  # Distance between center of wheels
        self._wheel_radius = 0.127   # Radius of wheel
        self._enc_pulses_per_turn = 5400.0
        self._qpps_left = 10000   # Max encoder ticks per seconds
        self._qpps_right = 10000
        self._last_odom_update = rospy.Time.now()
        self._distance_per_encoder_pulse = 2 * math.pi * self._wheel_radius / self._enc_pulses_per_turn
        self._traction_factor = 1.0

        # Odometry values
        self._odom_position_x = 0.0
        self._odom_position_y = 0.0
        self._odom_orientation_theta = 0.0

        # ROS Publishers
        self._pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        self._pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

        # ROS Subscribers
        self._twist_sub = rospy.Subscriber("/cmd_vel", Twist, self._twist_cb, queue_size=1)
        self._estop_on_sub = rospy.Subscriber("/soft_estop/enable", Bool, self._eStopCB, queue_size=1)
        self._estop_off_sub = rospy.Subscriber("/soft_estop/reset", Bool, self._eStopResetCB, queue_size=1)
        # ROS Timers
        rospy.Timer(rospy.Duration(self._diag_frequency), self._diag_cb)
        rospy.Timer(rospy.Duration(self._odom_frequency), self._odom_cb)
        rospy.Timer(rospy.Duration(self._cmd_vel_timeout), self._cmd_vel_timeout_cb)

        # Initialize Roboclaw Serial
        self._roboclaw = Roboclaw(self._port, self._baud)
        if not self._roboclaw.Open():
            rospy.logfatal('Could not open serial at ' + self._port)

        # Get Roboclaw Firmware Version
        self._firmware_version = self._roboclaw.ReadVersion(self._address)[1].replace('\n', '')

        # Reset Encoders
        self._roboclaw.ResetEncoders(self._address)

    def get_battery_voltage(self):
        self._battery_voltage = self._roboclaw.ReadMainBatteryVoltage(self._address)[1] / 10.0

    def get_motor_current(self):
        (res, m1_current, m2_current) = self._roboclaw.ReadCurrents(self._address)
        if res:
            self._left_motor_current = m1_current / 100.0
            self._right_motor_current = m2_current / 100.0

    def get_V_PID(self):
        (res, p, i, d, qpps) = self._roboclaw.ReadM1VelocityPID(self._address)
        if res:
            self._m1_v_p = p
            self._m1_v_i = i
            self._m1_v_d = d
            self._m1_v_qpps = qpps

        (res, p, i, d, qpps) = self._roboclaw.ReadM2VelocityPID(self._address)
        if res:
            self._m2_v_p = p
            self._m2_v_i = i
            self._m2_v_d = d
            self._m2_v_qpps = qpps

    def set_m1_v_pid(self, p, i, d, qpps):
        self._roboclaw.SetM1VelocityPID(self._address, p, i, d, qpps)

    def set_m2_v_pid(self, p, i, d, qpps):
        self._roboclaw.SetM2VelocityPID(self._address, p, i, d, qpps)

    def init_motor_controller(self):
        self._firmware_version = self._roboclaw.ReadVersion(self._address)
        if self._v_pid_overwrite:
            self.set_m1_v_pid(self._m1_v_p, self._m1_v_i, self._m1_v_d, self._m1_v_qpps)
            self.set_m2_v_pid(self._m2_v_p, self._m2_v_i, self._m2_v_d, self._m2_v_qpps)
        else:
            self.get_V_PID()

    def set_effort(self, left_effort, right_effort):
        self._roboclaw.ForwardBackwardM1(self._address, left_effort)
        self._roboclaw.ForwardBackwardM2(self._address, right_effort)

    def set_velocities(self, left_velocity, right_velocity):
        self._roboclaw.SpeedM1M2(self._address, left_velocity, right_velocity)

    def spin(self):
        rospy.spin()

    def _twist_cb(self, cmd):
        if self.e_stop_on_:
            self._left_effort, self._right_effort = 64, 64
        else:
            self._left_effort, self._right_effort = self._twist_to_esc_effort(cmd.linear.x, cmd.angular.z)
        self._serial_lock.acquire()
        self._last_cmd_vel_received = rospy.get_rostime()
        self.set_effort(self._left_effort, self._right_effort)
        self._serial_lock.release()

    def _twist_to_esc_effort(self, linear_rate, angular_rate):
        if linear_rate > self._max_vel:
            linear_rate = self._max_vel
        if angular_rate > self._max_turn:
            angular_rate = self._max_turn

        left_ = (linear_rate - 0.5 * self._wheel_base * angular_rate) / self._wheel_radius
        right_ = (linear_rate + 0.5 * self._wheel_base * angular_rate) / self._wheel_radius

        return (self.speed_to_effort(left_), self.speed_to_effort(right_))

    def _twist_to_esc_veloctity(self, linear_rate, angular_rate):
        if linear_rate > self._max_vel:
            linear_rate = self._max_vel
        if angular_rate > self._max_turn:
            angular_rate = self._max_turn

        left_ = (linear_rate - 0.5 * self._wheel_base * angular_rate) / self._wheel_radius
        right_ = (linear_rate + 0.5 * self._wheel_base * angular_rate) / self._wheel_radius

        return (self.speed_to_pulse_rate(left_), self.speed_to_pulse_rate(right_))

    def speed_to_effort(self, speed):
        if abs(speed) > self._max_vel:
            speed = self._max_vel if speed >= 0 else -self._max_vel
        effort = int(63 * (self._effort_coef * speed)) + 64
        if effort < 0:
            effort = 0
        if effort > 127:
            effort = 127

        return effort

    def speed_to_pulse_rate(self, speed):
        pulse_rate = speed / (2.0 * math.pi * self._wheel_radius) * self._enc_pulses_per_turn
        return int(pulse_rate)

    def _cmd_vel_timeout_cb(self, event):
        self._serial_lock.acquire()
        now = rospy.get_rostime()
        if now.to_sec() - self._last_cmd_vel_received.to_sec() > self._cmd_vel_timeout:
            self.set_effort(64, 64)
        self._serial_lock.release()

    def _diagnostics(self):
        self.get_battery_voltage()
        self.get_motor_current()

    def _diag_cb(self, event):
        # rospy callbacks are not thread safe.  This prevents serial interference
        self._serial_lock.acquire()
        self._diagnostics()
        self._serial_lock.release()
        darr = DiagnosticArray()
        darr.header.stamp = rospy.get_rostime()
        darr.status = [
            DiagnosticStatus(name='Firmware Version', message=str(self._firmware_version)),
            DiagnosticStatus(name='Left Motor Current', message='{CURRENT}A'.format(CURRENT=str(self._left_motor_current))),
            DiagnosticStatus(name='Right Motor Current', message='{CURRENT}A'.format(CURRENT=str(self._right_motor_current))),
            DiagnosticStatus(name='Battery Voltage', message='{VOLTAGE}V'.format(VOLTAGE=str(self._battery_voltage)))
        ]
        self._pub_diag.publish(darr)

    def _odom_cb(self, msg):
        self._serial_lock.acquire()
        left_speed = self._distance_per_encoder_pulse * self._roboclaw.ReadISpeedM1(self._address)[1]
        right_speed = self._distance_per_encoder_pulse * self._roboclaw.ReadISpeedM2(self._address)[1]
        self._serial_lock.release()

        update_time = rospy.Time.now()
        dt = (update_time - self._last_odom_update).to_sec()
        self._last_odom_update = update_time

        linear_vel = 0.5 * (left_speed + right_speed)
        turn_vel = right_speed - left_speed
        alpha_dot = turn_vel / self._wheel_base

        self._odom_position_x += dt * math.cos(self._odom_orientation_theta) * linear_vel
        self._odom_position_y += dt * math.sin(self._odom_orientation_theta) * linear_vel
        self._odom_orientation_theta += dt * alpha_dot

        odom_msg = Odometry()
        odom_msg.child_frame_id = self._base_link_frame
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.header.stamp = rospy.Time.now()

        quat =  quaternion_from_euler(0, 0, self._odom_orientation_theta)

        odom_msg.pose.pose.position.x = self._odom_position_x
        odom_msg.pose.pose.position.y = self._odom_position_y
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.pose.covariance[0] = 1e-2
        odom_msg.pose.covariance[7] = 1e-2
        odom_msg.pose.covariance[35] = 1e-2

        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = alpha_dot
        odom_msg.twist.covariance[0] = 1e-3
        odom_msg.twist.covariance[35] = 1e-4
        self._pub_odom.publish(odom_msg)

    def _eStopCB(self, estopstate):
        self._global_var_lock.acquire()
        if estopstate.data:
            self.e_stop_on_ = True
        self._global_var_lock.release()

    def _eStopResetCB(self, estopstate):
        self._global_var_lock.acquire()
        if estopstate.data:
            self.e_stop_on_ = False
        self._global_var_lock.release()


if __name__ == '__main__':
    rz = RoverZeroNode()
    rz.spin()
