#!/usr/bin/env python
from roboclaw_driver.roboclaw import Roboclaw
import rospy
from geometry_msgs.msg import Twist, Quaternion
from threading import Lock
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import PyKDL
import math
from std_msgs.msg import Bool

class RoverZeroNode:
    def __init__(self):
        self._node = rospy.init_node('Rover_Zero_Controller', anonymous=True)

        # ROS params
        self._port = rospy.get_param('~dev', '/dev/ttyACM0')
        self._address = rospy.get_param('~address', 128)
        self._baud = rospy.get_param('~baud', 115200)
        self._max_vel = rospy.get_param('~max_vel', 5.0)
        self._max_turn_rate = rospy.get_param('~max_turn_rate', 6.28)
        self._duty_coef = rospy.get_param('~speed_to_duty_coef', 1.02)
        self._diag_frequency = rospy.get_param('~diag_frequency_hz', 1.0)
        self._motor_cmd_frequency = rospy.get_param('~motor_cmd_frequency_hz', 30.0)
        self._odom_frequency = rospy.get_param('~odom_frequency_hz', 30.0)
        self._cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout', 0.5)
        self._encoder_odom_enabled = rospy.get_param('~enable_encoder_odom', False)
        self._esc_feedback_controls_enabled = rospy.get_param('~enable_esc_feedback_controls', False)
        self._v_pid_overwrite = rospy.get_param('~v_pid_overwrite', False)
        self._save_motor_controller_settings = rospy.get_param('~save_motor_controller_settings', False)
        self._m1_v_p = rospy.get_param('~m1_v_p', 3.00)
        self._m1_v_i = rospy.get_param('~m1_v_i', 0.35)
        self._m1_v_d = rospy.get_param('~m1_v_d', 0.00)
        self._m1_v_qpps = int(rospy.get_param('~m1_v_qpps', 10000))
        self._m2_v_p = rospy.get_param('~m2_v_p', 3.00)
        self._m2_v_i = rospy.get_param('~m2_v_i', 0.35)
        self._m2_v_d = rospy.get_param('~m2_v_d', 0.00)
        self._m2_v_qpps = int(rospy.get_param('~m2_v_qpps', 10000))
        self._encoder_pulses_per_turn = rospy.get_param('~encoder_pulses_per_turn', 5400.0)
        self._left_motor_max_current = rospy.get_param('~left_motor_max_current', 5.0)
        self._right_motor_max_current = rospy.get_param('~right_motor_max_current', 5.0)
        self._active_brake_timeout = rospy.get_param('~active_brake_timeout', 1.0)
        self._odom_frame = rospy.get_param('~odom_frame', "odom")
        self._base_link_frame = rospy.get_param('~base_link_frame', "base_link")
        self._wheel_base = rospy.get_param('~wheel_base', 0.358775)  # Distance between center of wheels
        self._wheel_radius = rospy.get_param('~wheel_radius', 0.127)   # Radius of wheel

        # Initialize Roboclaw Serial
        self._roboclaw = Roboclaw(self._port, self._baud)
        if not self._roboclaw.Open():
            rospy.logfatal('Could not open serial at ' + self._port)
            exit(1)

        self.verify_ros_parameters()

        # Class Variables
        self._left_motor_speed = 0.0
        self._right_motor_speed = 0.0
        self._serial_lock = Lock()
        self._variable_lock = Lock()
        self._last_cmd_vel_received = rospy.get_rostime()
        self._estop_on_ = False
        self._last_odom_update = rospy.Time.now()
        self._active_brake_start_time = None
        self._active_brake_enabled = False
        self._distance_per_encoder_pulse = 2 * math.pi * self._wheel_radius / self._encoder_pulses_per_turn

        # Diagnostic Parameters
        self._firmware_version = None
        self._left_motor_current = None
        self._right_motor_current = None
        self._battery_voltage = None
        self._controller_error = None

        # Odometry values
        self._odom_position_x = 0.0
        self._odom_position_y = 0.0
        self._odom_orientation_theta = 0.0

        # ROS Publishers
        if self._diag_frequency > 0.0:
            self._pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        if self._encoder_odom_enabled and self._odom_frequency > 0.0:
            self._pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

        # ROS Subscribers
        self._twist_sub = rospy.Subscriber("/cmd_vel", Twist, self._twist_cmd_cb, queue_size=1)
        self._estop_enable_sub = rospy.Subscriber("/soft_estop/enable", Bool, self._estop_enable_cb, queue_size=1)
        self._estop_reset_sub = rospy.Subscriber("/soft_estop/reset", Bool, self._estop_reset_cb, queue_size=1)

        # ROS Timers
        if self._diag_frequency > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self._diag_frequency), self._diag_cb)
        rospy.Timer(rospy.Duration(1.0 / self._motor_cmd_frequency), self._motor_cmd_cb)
        if self._encoder_odom_enabled and self._odom_frequency > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self._odom_frequency), self._odom_cb)
        if self._cmd_vel_timeout > 0.0:
            rospy.Timer(rospy.Duration(self._cmd_vel_timeout), self._cmd_vel_timeout_cb)

        # Get Roboclaw Firmware Version
        self._firmware_version = self._roboclaw.ReadVersion(self._address)[1].replace('\n', '')

        self._configure_motor_controller()

        # Reset Encoders
        self._roboclaw.ResetEncoders(self._address)

    def _configure_motor_controller(self):
        self._firmware_version = self._roboclaw.ReadVersion(self._address)

        self._roboclaw.SetM1MaxCurrent(self._address, int(100 * self._left_motor_max_current))
        self._roboclaw.SetM2MaxCurrent(self._address, int(100 * self._right_motor_max_current))

        if self._esc_feedback_controls_enabled:
            if self._v_pid_overwrite:
                self._roboclaw.SetM1VelocityPID(self._address, self._m1_v_p, self._m1_v_i, self._m1_v_d, self._m1_v_qpps)
                self._roboclaw.SetM2VelocityPID(self._address, self._m2_v_p, self._m2_v_i, self._m2_v_d, self._m2_v_qpps)
            else:
                self._get_V_PID()

        if self._save_motor_controller_settings:
            self._roboclaw.WriteNVM(self._address)

    def _get_V_PID(self):
        (res, p, i, d, qpps) = self._roboclaw.ReadM1VelocityPID(self._address)
        if res:
            self._m1_v_p = p
            self._m1_v_i = i
            self._m1_v_d = d
            self._m1_v_qpps = qpps
            rospy.loginfo('Motor 1 (P, I, D, QPPS): %f, %f, %f, %d', p, i, d, qpps)

        (res, p, i, d, qpps) = self._roboclaw.ReadM2VelocityPID(self._address)
        if res:
            self._m2_v_p = p
            self._m2_v_i = i
            self._m2_v_d = d
            self._m2_v_qpps = qpps
            rospy.loginfo('Motor 2 (P, I, D, QPPS): %f, %f, %f, %d', p, i, d, qpps)

    def verify_ros_parameters(self):
        fatal_misconfiguration = False

        if self._max_vel <= 0.0:
            rospy.logfatal('Maximum velocity (max_vel) must be greater than 0.')
            fatal_misconfiguration = True

        if self._max_turn_rate <= 0.0:
            rospy.logfatal('Maximum turn rate (max_turn_rate) must be greater than 0.')
            fatal_misconfiguration = True

        if self._duty_coef <= 0.0:
            rospy.logfatal('The wheel velocity to motor duty coefficient (speed_to_duty_coef) must be greater than 0.')
            fatal_misconfiguration = True

        if self._encoder_pulses_per_turn <= 0.0:
            rospy.logfatal('Encoder pulses per turn (encoder_pulses_per_turn) must be greater than 0.')
            fatal_misconfiguration = True

        if self._motor_cmd_frequency < 1.0:
            rospy.logfatal('Motor command frequency (motor_cmd_frequency_hz) must be greater than or equal to 1Hz.')
            fatal_misconfiguration = True
        elif self._motor_cmd_frequency < 5.0:
            rospy.logwarn('Motor command frequency (motor_cmd_frequency_hz) is low.  It is suggested that motor command update rate of at least 5Hz.')

        if self._cmd_vel_timeout <= 0.0:
            rospy.logwarn('cmd_vel timeout (cmd_vel_timeout) disabled.  Robot will execute last cmd_vel message forever.')

        if not self._encoder_odom_enabled:
            rospy.logwarn('Encoder odometry are not enabled (enable_encoder_odom).')

        if self._esc_feedback_controls_enabled:
            if self._v_pid_overwrite:
                if (self._m1_v_p == 0.0 and self._m1_v_i == 0.0 and self._m1_v_d == 0.0) or self._m1_v_qpps <= 0:
                    rospy.logfatal('Invalid user specified PID parameters for left motor.')
                    fatal_misconfiguration = True
            else:
                (res, p, i, d, qpps) = self._roboclaw.ReadM1VelocityPID(self._address)
                if (p == 0.0 and i == 0.0 and d == 0.0) or qpps <= 0:
                    rospy.logfatal('Invalid PID parameters for left motor saved on Roboclaw.')
                    fatal_misconfiguration = True

        if self._esc_feedback_controls_enabled:
            if self._v_pid_overwrite:
                if (self._m2_v_p == 0.0 and self._m2_v_i == 0.0 and self._m2_v_d == 0.0) or self._m2_v_qpps <= 0:
                    rospy.logfatal('Invalid user specified PID parameters for right motor.')
                    fatal_misconfiguration = True
            else:
                (res, p, i, d, qpps) = self._roboclaw.ReadM2VelocityPID(self._address)
                if (p == 0.0 and i == 0.0 and d == 0.0) or qpps <= 0:
                    rospy.logfatal('Invalid PID parameters for right motor saved on Roboclaw.')
                    fatal_misconfiguration = True

        if not self._esc_feedback_controls_enabled:
            rospy.logwarn('PID feedback controls are not enabled (enable_esc_feedback_controls), running in duty cycles mode.')
            rospy.logwarn('BEWARE: In duty cycle mode if the serial connection is lost it will indefitely execute the last command sent to the Roboclaw.')

        if self._save_motor_controller_settings:
            rospy.loginfo('New motor controller settings will be saved to Roboclaw eeprom')

        if self._left_motor_max_current <= 0.0:
            rospy.logfatal('Maximum current for left motor (left_motor_max_current) must be greater than 0')
            fatal_misconfiguration = True

        if self._right_motor_max_current <= 0.0:
            rospy.logfatal('Maximum current for right motor (right_motor_max_current) must be greater than 0')
            fatal_misconfiguration = True

        if self._active_brake_timeout <= 0:
            rospy.logwarn('Active breaking timeout (active_brake_timeout) is disabled. If PID control is enabled and active breaking is disabled when idle, passive current may be applied motors causing potential motor overheating or reduced battery life.')

        if not isinstance(self._odom_frame, str) or not self._odom_frame:
            rospy.logfatal('Invalid baselink frame: \'%s\'', self._odom_frame)
            fatal_misconfiguration = True

        if not isinstance(self._base_link_frame, str) or not self._base_link_frame:
            rospy.logfatal('Invalid baselink frame: \'%s\'', self._base_link_frame)
            fatal_misconfiguration = True

        if self._wheel_base <= 0.0:
            rospy.logfatal("Wheel base (wheel_base) must be greater than 0")
            fatal_misconfiguration = True

        if self._wheel_radius <= 0.0:
            rospy.logfatal("Wheel radius (wheel_radius) must be greater than 0")
            fatal_misconfiguration = True

        if fatal_misconfiguration:
            rospy.signal_shutdown('Fatal Misconfiguration')
            exit(1)

    def _twist_cmd_cb(self, cmd):
        self._last_cmd_vel_received = rospy.Time.now()
        if self._estop_on_:
            self._left_motor_speed = 0.0
            self._right_motor_speed = 0.0
        else:
            self._left_motor_speed, self._right_motor_speed = \
                self._twist_to_wheel_velocities(cmd.linear.x, cmd.angular.z)

    def _estop_enable_cb(self, estopstate):
        if estopstate.data and not self._estop_on_:
            self._variable_lock.acquire()
            self._estop_on_ = True
            self._variable_lock.release()

    def _estop_reset_cb(self, estopstate):
        if estopstate.data and self._estop_on_:
            self._variable_lock.acquire()
            self._estop_on_ = False
            self._variable_lock.release()

    def _twist_to_wheel_velocities(self, linear_rate, angular_rate):
        if linear_rate > self._max_vel:
            linear_rate = self._max_vel
        if angular_rate > self._max_turn_rate:
            angular_rate = self._max_turn_rate

        left_ = (linear_rate - 0.5 * self._wheel_base * angular_rate)
        right_ = (linear_rate + 0.5 * self._wheel_base * angular_rate)

        return left_, right_

    def _motor_cmd_cb(self, event):
        self._variable_lock.acquire()
        left_speed, right_speed = self._left_motor_speed, self._right_motor_speed
        self._variable_lock.release()

        if self._esc_feedback_controls_enabled:
            if left_speed != 0.0 or right_speed != 0.0:
                self._active_brake_enabled = False
                self._active_brake_start_time = None
            else:
                if self._active_brake_enabled is False and self._active_brake_start_time is None:
                    self._active_brake_enabled = True
                    self._active_brake_start_time = rospy.Time.now()

            if (self._active_brake_enabled and self._active_brake_start_time is not None and
                    rospy.Time.now().to_sec() - self._active_brake_start_time.to_sec() > self._active_brake_timeout):
                self.send_motor_duty(0, 0)
                return

            left_cmd, right_cmd = self.speed_to_pulse_rate(left_speed), self.speed_to_pulse_rate(right_speed)
            self.send_motor_velocities(left_cmd, right_cmd)

        else:
            left_cmd, right_cmd = self.speed_to_duty(left_speed), self.speed_to_duty(right_speed)
            self.send_motor_duty(left_cmd, right_cmd)

    def speed_to_pulse_rate(self, speed):
        pulse_rate = speed / self._distance_per_encoder_pulse
        return int(pulse_rate)

    def speed_to_duty(self, speed):
        duty = int(32768 * (self._duty_coef * speed))
        if duty < -32768:
            duty = -32768
        if duty > 32768:
            duty = 32768

        return duty

    def send_motor_duty(self, left_duty, right_duty):
        self._serial_lock.acquire()
        self._roboclaw.DutyM1M2(self._address, left_duty, right_duty)
        self._serial_lock.release()

    def send_motor_velocities(self, left_velocity, right_velocity):
        """The Roboclaw does not have a serial timeout functionality. To make the robot safer we use the velocity with distance command to limit the distance the robot travels if the controller loses connection to the driver node"""
        dt = 5 / self._motor_cmd_frequency
        dt = 2.0 if dt > 2.0 else dt
        self._serial_lock.acquire()
        self._roboclaw.SpeedDistanceM1M2(self._address, left_velocity, int(dt * left_velocity),
                                         right_velocity, int(dt * right_velocity), 1)
        self._serial_lock.release()

    def _cmd_vel_timeout_cb(self, event):
        now = rospy.get_rostime()
        if now.to_sec() - self._last_cmd_vel_received.to_sec() > self._cmd_vel_timeout:
            self._variable_lock.acquire()
            self._left_motor_speed, self._right_motor_speed = 0.0, 0.0
            self._variable_lock.release()

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

        quat = [0, 0, 0, 0]
        PyKDL.RPY(0, 0, self._odom_orientation_theta).GetQuaternion(quat[0], quat[1], quat[2], quat[3])

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

    def _diag_cb(self, event):
        # rospy callbacks are not thread safe.  This prevents serial interference
        self._diagnostics_update()
        darr = DiagnosticArray()
        darr.header.stamp = rospy.get_rostime()
        darr.status = [
            DiagnosticStatus(name="Roboclaw Driver",
                             message="OK",
                             hardware_id="none",
                             values=[KeyValue(key='Firmware Version', value=str(self._firmware_version)),
                                     KeyValue(key='Left Motor Max Current', value='{MAX_CURRENT} A'.format(MAX_CURRENT=str(self._left_motor_max_current))),
                                     KeyValue(key='Left Motor Current', value='{CURRENT} A'.format(CURRENT=str(self._left_motor_current))),
                                     KeyValue(key='Left Motor Max Current', value='{MAX_CURRENT} A'.format(MAX_CURRENT=str(self._right_motor_max_current))),
                                     KeyValue(key='Right Motor Current', value='{CURRENT} A'.format(CURRENT=str(self._right_motor_current))),
                                     KeyValue(key='Battery Voltage', value='{VOLTAGE}V'.format(VOLTAGE=str(self._battery_voltage))),
                                     KeyValue(key='Drive Mode', value='Closed Loop Control' if self._esc_feedback_controls_enabled else 'Open Loop Control')
                                     ]
                             )
        ]

        self._pub_diag.publish(darr)

    def _diagnostics_update(self):
        self.get_battery_voltage()
        self.get_motor_current()

    def get_battery_voltage(self):
        self._serial_lock.acquire()
        self._battery_voltage = self._roboclaw.ReadMainBatteryVoltage(self._address)[1] / 10.0
        self._serial_lock.release()

    def get_motor_current(self):
        self._serial_lock.acquire()
        (res, m1_current, m2_current) = self._roboclaw.ReadCurrents(self._address)
        self._serial_lock.release()

        if res:
            self._left_motor_current = m1_current / 100.0
            self._right_motor_current = m2_current / 100.0

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rz = RoverZeroNode()
    rz.spin()
