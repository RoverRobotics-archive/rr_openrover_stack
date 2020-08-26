#include "ros/ros.h"

#include <fcntl.h>
#include <termios.h>
#include <ctime>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/ioctl.h>

#include <sensor_msgs/Joy.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>
#include "nav_msgs/Odometry.h"
#include "rr_openrover_driver_msgs/RawRrOpenroverDriverFastRateData.h"
#include "rr_openrover_driver_msgs/RawRrOpenroverDriverMedRateData.h"
#include "rr_openrover_driver_msgs/RawRrOpenroverDriverSlowRateData.h"
#include "rr_openrover_driver_msgs/SmartBatteryStatus.h"

#include "rr_openrover_driver/openrover.hpp"

namespace openrover
{
OpenRover::OpenRover(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh)
  , nh_priv_(nh_priv)
  , port_("/dev/ttyUSB0")
  , serial_baud_rate_(57600)
  , use_legacy_(false)
  , fast_rate_hz_(10.0)                                                        // can increase to 60Hz for TX2
  , medium_rate_hz_(2.0)
  , slow_rate_hz_(1.0)
  , motor_speeds_commanded_{ MOTOR_NEUTRAL, MOTOR_NEUTRAL, MOTOR_NEUTRAL }  // default motor commands to neutral
  , timeout_(0.2)                                                           // in seconds
  , publish_fast_rate_values_(false)
  , publish_med_rate_values_(false)
  , publish_slow_rate_values_(false)
  , is_serial_coms_open_(false)
  , closed_loop_control_on_(false)
  , pidGains_(40, 100, 0)
  , left_vel_commanded_(0)
  , right_vel_commanded_(0)
  , left_vel_filtered_(0)
  , right_vel_filtered_(0)
  , left_vel_measured_(0)
  , right_vel_measured_(0)
  , e_stop_on_(false)
  , LEFT_MOTOR_INDEX_(0)
  , RIGHT_MOTOR_INDEX_(1)
  , FLIPPER_MOTOR_INDEX_(2)
  , l_pid_csv_file_("")
  , r_pid_csv_file_("")
{
  ROS_INFO("Initializing openrover driver.");
}

bool OpenRover::start()
{
  if (!setupRobotParams())
  {
    ROS_WARN("Failed to setup Robot parameters.");
    return false;
  }

  ROS_INFO("%f, %f, %f", pidGains_.Kp, pidGains_.Ki, pidGains_.Kd);

  if (l_fs_.is_open()) {
    left_controller_ = OdomControl(closed_loop_control_on_, pidGains_, MOTOR_SPEED_MAX, MOTOR_SPEED_MIN, &l_fs_);
  }

  if (r_fs_.is_open()) {
    right_controller_ = OdomControl(closed_loop_control_on_, pidGains_, MOTOR_SPEED_MAX, MOTOR_SPEED_MIN, &r_fs_);
  }

  left_controller_.start(closed_loop_control_on_, pidGains_, MOTOR_SPEED_MAX, MOTOR_SPEED_MIN);
  right_controller_.start(closed_loop_control_on_, pidGains_, MOTOR_SPEED_MAX, MOTOR_SPEED_MIN);

  serial_fast_buffer_.reserve(5 * FAST_SIZE);      // reserve space for 5 sets of FAST rate data
  serial_medium_buffer_.reserve(5 * MEDIUM_SIZE);  // reserve space for 5 sets of Medium rate data
  serial_slow_buffer_.reserve(5 * SLOW_SIZE);      // reserve space for 5 sets of Slow rate data
  serial_fan_buffer_.reserve(5);                   // reserve space for 5 sets of Fan commands

  ROS_INFO("Creating Publishers and Subscribers");
  // WallTimers simplify the timing of updating parameters by reloading serial buffers at specified rates.
  // without them the serial buffers will never be loaded with new commands
  fast_timer = nh_priv_.createWallTimer(ros::WallDuration(1.0 / fast_rate_hz_), &OpenRover::robotDataFastCB, this);
  medium_timer = nh_priv_.createWallTimer(ros::WallDuration(1.0 / medium_rate_hz_), &OpenRover::robotDataMediumCB, this);
  slow_timer = nh_priv_.createWallTimer(ros::WallDuration(1.0 / slow_rate_hz_), &OpenRover::robotDataSlowCB, this);
  timeout_timer = nh_priv_.createWallTimer(ros::WallDuration(timeout_), &OpenRover::timeoutCB, this, true);


  if (!(nh_priv_.getParam("use_legacy", use_legacy_)))
  {
    ROS_WARN("Failed to retrieve drive_type from parameter.Defaulting to %s", use_legacy_ ? "true" : "false");
  }

  if(use_legacy_) {
    fast_rate_pub = nh_priv_.advertise<rr_openrover_driver_msgs::RawRrOpenroverDriverFastRateData>("raw_fast_rate_data",
                                                                                                   1);
    medium_rate_pub = nh_priv_.advertise<rr_openrover_driver_msgs::RawRrOpenroverDriverMedRateData>("raw_med_rate_data",
                                                                                                    1);
    slow_rate_pub = nh_priv_.advertise<rr_openrover_driver_msgs::RawRrOpenroverDriverSlowRateData>("raw_slow_rate_data",
                                                                                                   1);
  }
  battery_status_a_pub = nh_priv_.advertise<rr_openrover_driver_msgs::SmartBatteryStatus>("battery_status_a", 1);
  battery_status_b_pub = nh_priv_.advertise<rr_openrover_driver_msgs::SmartBatteryStatus>("battery_status_b", 1);
  battery_state_of_charge_pub = nh_priv_.advertise<std_msgs::Int32>("battery_state_of_charge", 1);
  odom_enc_pub = nh_priv_.advertise<nav_msgs::Odometry>("odom_encoder", 1);
  is_charging_pub = nh_priv_.advertise<std_msgs::Bool>("charging", 1);

  motor_speeds_pub = nh_priv_.advertise<std_msgs::Int32MultiArray>("motor_speeds_commanded", 1);
  vel_calc_pub = nh_priv_.advertise<std_msgs::Float32MultiArray>("vel_calc_pub", 1);

  joy_sub = nh_priv_.subscribe("/cmd_vel/managed", 1, &OpenRover::joyCB, this);
  cmd_vel_sub = nh_priv_.subscribe("/cmd_vel/managed", 1, &OpenRover::cmdVelCB, this);
  fan_speed_sub = nh_priv_.subscribe("/rr_openrover_driver/fan_speed", 1, &OpenRover::fanSpeedCB, this);
  e_stop_sub = nh_priv_.subscribe("/soft_estop/enable", 1, &OpenRover::eStopCB, this);
  e_stop_reset_sub = nh_priv_.subscribe("/soft_estop/reset", 1, &OpenRover::eStopResetCB, this);

  return true;
}

bool OpenRover::setupRobotParams()
{  // Get ROS params and save them to class variables

  if (!(nh_priv_.getParam("port", port_)))
  {
    ROS_WARN("Failed to retrieve port from parameter server.Defaulting to %s", port_.c_str());
  }

  if (!(openComs()))
  {
    is_serial_coms_open_ = false;
    ROS_ERROR("Failed to start serial communication.");
    return false;
  }

  if (!(nh_priv_.getParam("fast_data_rate", fast_rate_hz_)))
  {
    ROS_WARN("Failed to retrieve fast_data_rate from parameter. Defaulting to 10");
  }

  if (!(nh_priv_.getParam("medium_data_rate", medium_rate_hz_)))
  {
    ROS_WARN("Failed to retrieve medium_data_rate from parameter. Defaulting to 2");
  }

  if (!(nh_priv_.getParam("slow_data_rate", slow_rate_hz_)))
  {
    ROS_WARN("Failed to retrieve slow_data_rate from parameter. Defaulting to 1");
  }


  if (!(nh_priv_.getParam("closed_loop_control_on", closed_loop_control_on_)))
  {
    ROS_WARN("Failed to retrieve closed_loop_control_on from parameter server. Defaulting to off.");
  }

  if (!(nh_priv_.getParam("timeout", timeout_)))
  {
    ROS_WARN("Failed to retrieve timeout from parameter server. Defaulting to %f s", timeout_);
  }

  if (!(nh_priv_.getParam("total_weight", total_weight_)))
  {
    ROS_WARN("Failed to retrieve total_weight from parameter server. Defaulting to %f lbs", total_weight_);
  }

  if (!(nh_priv_.getParam("drive_type", drive_type_)))
  {
    ROS_WARN("Failed to retrieve drive_type from parameter.Defaulting to %s", drive_type_.c_str());
  }

  if (!(nh_priv_.getParam("Kp", pidGains_.Kp)))
  {
    ROS_WARN("Failed to retrieve Kp from parameter.Defaulting to %f", pidGains_.Kp);
  }
  else{
    ROS_INFO("Kp: %f", pidGains_.Kp);
  }

  if (!(nh_priv_.getParam("Ki", pidGains_.Ki)))
  {
    ROS_WARN("Failed to retrieve Ki from parameter.Defaulting to %f", pidGains_.Ki);
  }
  else{
    ROS_INFO("Ki: %f", pidGains_.Ki);
  }

  if (!(nh_priv_.getParam("Kd", pidGains_.Kd)))
  {
    ROS_WARN("Failed to retrieve Kd from parameter.Defaulting to %f", pidGains_.Kd);
  }
  else{
    ROS_INFO("Kd: %f", pidGains_.Kd);
  }

  if (!(nh_priv_.getParam("left_motor_pid_csv", l_pid_csv_file_)))
  {
    ROS_INFO("Not logging left motor PID");
  }
  else{
    ROS_INFO("Recording left motor PID data to %s", l_pid_csv_file_.c_str());
  }

  if (!(nh_priv_.getParam("right_motor_pid_csv", r_pid_csv_file_)))
  {
    ROS_INFO("Not logging right motor PID");
  }
  else{
    ROS_INFO("Recording right motor PID data to %s", r_pid_csv_file_.c_str());
  }

  if (!l_pid_csv_file_.empty()){
    l_fs_.open(l_pid_csv_file_, std::ofstream::out);
    if(!l_fs_.is_open()){
      ROS_WARN("Could not open file: %s", l_pid_csv_file_.c_str());
    }
  }

  if (!r_pid_csv_file_.empty()){
    r_fs_.open(r_pid_csv_file_, std::ofstream::out);
    if(!r_fs_.is_open()){
      ROS_WARN("Could not open file: %s", r_pid_csv_file_.c_str());
    }
  }

  if (drive_type_ == (std::string) "2wd")
  {
    ROS_INFO("2wd parameters loaded.");
    odom_encoder_coef_ = ODOM_ENCODER_COEF_2WD;
    odom_axle_track_ = ODOM_AXLE_TRACK_2WD;
    odom_angular_coef_ = ODOM_ANGULAR_COEF_2WD;
    odom_traction_factor_ = ODOM_TRACTION_FACTOR_2WD;

    motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_2WD_HS;
    motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_2WD_HS;

    motor_speed_flipper_coef_ = MOTOR_FLIPPER_COEF;
    motor_speed_deadband_ = MOTOR_DEADBAND;
  }
  else if (drive_type_ == (std::string) "4wd")
  {
    ROS_INFO("4wd parameters loaded.");
    odom_encoder_coef_ = ODOM_ENCODER_COEF_4WD;
    odom_axle_track_ = ODOM_AXLE_TRACK_4WD;
    odom_angular_coef_ = ODOM_ANGULAR_COEF_4WD;
    odom_traction_factor_ = ODOM_TRACTION_FACTOR_4WD;

    motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_4WD_HS;
    motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_4WD_HS;
    
    motor_speed_flipper_coef_ = MOTOR_FLIPPER_COEF;
    motor_speed_deadband_ = MOTOR_DEADBAND;
  }
  else if (drive_type_ == (std::string) "flippers")
  {
    ROS_INFO("flipper parameters loaded.");
    odom_encoder_coef_ = ODOM_ENCODER_COEF_F;
    odom_axle_track_ = ODOM_AXLE_TRACK_F;
    odom_angular_coef_ = ODOM_ANGULAR_COEF_F;
    odom_traction_factor_ = ODOM_TRACTION_FACTOR_F;

    motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_F_HS;
    motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_F_HS;
    
    motor_speed_flipper_coef_ = MOTOR_FLIPPER_COEF;
    motor_speed_deadband_ = MOTOR_DEADBAND;
  }
  else
  {
    ROS_WARN("Unclear ROS param drive_type. Defaulting to flippers params.");
    odom_encoder_coef_ = ODOM_ENCODER_COEF_F;
    odom_axle_track_ = ODOM_AXLE_TRACK_F;
    odom_angular_coef_ = ODOM_ANGULAR_COEF_F;
    odom_traction_factor_ = ODOM_TRACTION_FACTOR_F;

    motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_F_HS;
    motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_F_HS;
    
    motor_speed_flipper_coef_ = MOTOR_FLIPPER_COEF;
    motor_speed_deadband_ = MOTOR_DEADBAND;
  }

  if (!(nh_priv_.getParam("traction_factor", odom_traction_factor_)))
  {
    ROS_WARN("Failed to retrieve traction_factor from parameter. Defaulting to %f", odom_traction_factor_);
    odom_traction_factor_ = 0.61;
  }

  if (!(nh_priv_.getParam("odom_covariance_0", odom_covariance_0_)))
  {
    ROS_WARN("Failed to retrieve odom_covariance_0 from parameter. Defaulting to 0.01");
    odom_covariance_0_ = 0.01;
  }

  if (!(nh_priv_.getParam("odom_covariance_35", odom_covariance_35_)))
  {
    ROS_WARN("Failed to retrieve odom_covariance_35 from parameter. Defaulting to 0.03");
    odom_covariance_35_ = 0.03;
  }

  ROS_INFO("Openrover parameters loaded:");
  ROS_INFO("port: %s", port_.c_str());
  ROS_INFO("drive_type: %s", drive_type_.c_str());
  ROS_INFO("timeout: %f s", timeout_);
  ROS_INFO("closed_loop_control_on: %i", closed_loop_control_on_);
  ROS_INFO("total_weight: %f kg", total_weight_);
  ROS_INFO("traction_factor: %f", odom_traction_factor_);
  ROS_INFO("odom_covariance_0: %f", odom_covariance_0_);
  ROS_INFO("odom_covariance_35: %f", odom_covariance_35_);
  ROS_INFO("fast_data_rate: %f hz", fast_rate_hz_);
  ROS_INFO("medium_data_rate: %f hz", medium_rate_hz_);
  ROS_INFO("slow_data_rate: %f hz", slow_rate_hz_);

  return true;
}

void OpenRover::robotDataSlowCB(const ros::WallTimerEvent& e)
{
  if (is_serial_coms_open_ && !publish_slow_rate_values_)
  {
    for (int i = 0; i < SLOW_SIZE; i++)
    {
      serial_slow_buffer_.push_back(ROBOT_DATA_INDEX_SLOW[i]);
    }
    publish_slow_rate_values_ = true;
  }
  return;
}

void OpenRover::robotDataMediumCB(const ros::WallTimerEvent& e)
{
  if (is_serial_coms_open_ && !publish_med_rate_values_)
  {
    for (int i = 0; i < MEDIUM_SIZE; i++)
    {
      serial_medium_buffer_.push_back(ROBOT_DATA_INDEX_MEDIUM[i]);
    }
    publish_med_rate_values_ = true;
  }
  return;
}

void OpenRover::robotDataFastCB(const ros::WallTimerEvent& e)
{
  if (is_serial_coms_open_ && !publish_fast_rate_values_)
  {
    for (int i = 0; i < FAST_SIZE; i++)
    {
      // Fill buffer with all the param2's defined as fast data
      // by the ROBOT_DATA_INDEX_FAST array
      serial_fast_buffer_.push_back(ROBOT_DATA_INDEX_FAST[i]);
    }
    publish_fast_rate_values_ = true;
  }
  else
  {
    ROS_WARN_DELAYED_THROTTLE(5, "Fast data rate too high. Consider reducing fast_data_rate param");
  }
  return;
}

void OpenRover::timeoutCB(const ros::WallTimerEvent& e)
{  // Timer goes off when a command isn't received soon enough. Sets motors to neutral values
  motor_speeds_commanded_[LEFT_MOTOR_INDEX_] = MOTOR_NEUTRAL;
  motor_speeds_commanded_[RIGHT_MOTOR_INDEX_] = MOTOR_NEUTRAL;
  motor_speeds_commanded_[FLIPPER_MOTOR_INDEX_] = MOTOR_NEUTRAL;
  left_vel_commanded_ = 0.0;
  right_vel_commanded_ = 0.0;

  return;
}

void OpenRover::fanSpeedCB(const std_msgs::Int32::ConstPtr& msg)
{
  if (is_serial_coms_open_ && (serial_fan_buffer_.size() == 0))
  {
    serial_fan_buffer_.push_back(msg->data);
  }
  // ROS_DEBUG("Fan Buffer size is %i, new data is %i", serial_fan_buffer_.size(), msg->data);
  return;
}

void OpenRover::joyCB(const sensor_msgs::Joy::ConstPtr& msg){
//Get joy_msg Trimmers button and increase trim
  joy_commands_ = *msg;
  
}

void OpenRover::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{  // converts from cmd_vel (m/s and radians/s) into motor speed commands
  cmd_vel_commanded_ = *msg;
  float left_motor_speed, right_motor_speed;
  int flipper_motor_speed;
  int motor_speed_deadband_scaled;
  double turn_rate = msg->angular.z;
  double linear_rate = msg->linear.x;
  double flipper_rate = msg->angular.y;
  if (turn_rate == 0){
    if(linear_rate > 0){
      linear_rate = trim;
    }else if(linear_rate <0){
      linear_rate = -trim;
    }
  }
  static bool prev_e_stop_state_ = false;

  double diff_vel_commanded = turn_rate / odom_angular_coef_ / odom_traction_factor_;

  right_vel_commanded_ = linear_rate + 0.5 * diff_vel_commanded;
  left_vel_commanded_ = linear_rate - 0.5 * diff_vel_commanded;

  timeout_timer.stop();

    if (e_stop_on_)
    {
        if (!prev_e_stop_state_)
        {
            prev_e_stop_state_ = true;
            ROS_WARN("Openrover driver - Soft e-stop on.");
        }
        motor_speeds_commanded_[LEFT_MOTOR_INDEX_] = MOTOR_NEUTRAL;
        motor_speeds_commanded_[RIGHT_MOTOR_INDEX_] = MOTOR_NEUTRAL;
        motor_speeds_commanded_[FLIPPER_MOTOR_INDEX_] = MOTOR_NEUTRAL;
        return;
    }
    else
    {
        if (prev_e_stop_state_)
        {
            prev_e_stop_state_ = false;
            ROS_INFO("Openrover driver - Soft e-stop off.");
        }
    }

  flipper_motor_speed = ((int)round(flipper_rate * motor_speed_flipper_coef_) + 125) % 250;

  motor_speeds_commanded_[FLIPPER_MOTOR_INDEX_] = (unsigned char)flipper_motor_speed;

  timeout_timer.start();
  return;
}

void OpenRover::eStopCB(const std_msgs::Bool::ConstPtr& msg)
{
    static bool prev_e_stop_state_ = false;

    // e-stop only trigger on the rising edge of the signal and only deactivates when reset
    if(msg->data && !prev_e_stop_state_)
    {
        e_stop_on_ = true;
    }

    prev_e_stop_state_ = msg->data;
    return;
}

void OpenRover::eStopResetCB(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        e_stop_on_ = false;
    }
    return;
}

void OpenRover::publishOdometry(float left_vel, float right_vel)
{  // convert encoder readings to real world values and publish as Odometry
  static double left_dist = 0;
  static double right_dist = 0;
  static double pos_x = 0;
  static double pos_y = 0;
  static double theta = 0;
  static double past_time = 0;
  double net_vel = 0;
  double diff_vel = 0;
  double alpha = 0;
  double dt = 0;
  tf2::Quaternion q_new;

  ros::Time ros_now_time = ros::Time::now();
  double now_time = ros_now_time.toSec();

  nav_msgs::Odometry odom_msg;

  dt = now_time - past_time;
  past_time = now_time;

  if (past_time != 0)
  {
    left_dist += left_vel * dt;
    right_dist += right_vel * dt;

    net_vel = 0.5 * (left_vel + right_vel);
    diff_vel = right_vel - left_vel;

    alpha = odom_angular_coef_ * diff_vel * odom_traction_factor_;

    pos_x = pos_x + net_vel * cos(theta) * dt;
    pos_y = pos_y + net_vel * sin(theta) * dt;
    theta = (theta + alpha * dt);

    q_new.setRPY(0, 0, theta);
    tf2::convert(q_new, odom_msg.pose.pose.orientation);
  }

  odom_msg.header.stamp = ros_now_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.twist.twist.linear.x = net_vel;
  odom_msg.twist.twist.angular.z = alpha;

  // If not moving, trust the encoders completely
  // otherwise set them to the ROS param
  if (net_vel == 0 && alpha == 0)
  {
    odom_msg.twist.covariance[0] = odom_covariance_0_ / 1e3;
    odom_msg.twist.covariance[7] = odom_covariance_0_ / 1e3;
    odom_msg.twist.covariance[35] = odom_covariance_35_ / 1e6;
  }
  else
  {
    odom_msg.twist.covariance[0] = odom_covariance_0_;
    odom_msg.twist.covariance[7] = odom_covariance_0_;
    odom_msg.twist.covariance[35] = odom_covariance_35_;
  }

  odom_msg.pose.pose.position.x = pos_x;
  odom_msg.pose.pose.position.y = pos_y;

  odom_enc_pub.publish(odom_msg);
  return;
}

void OpenRover::publishWheelVels()
{  // Update to publish from OdomControl
  static ros::Time ros_start_time = ros::Time::now();
  ros::Time ros_now_time = ros::Time::now();
  double run_time = (ros_now_time - ros_start_time).toSec();
  std_msgs::Float32MultiArray vel_vec;

  vel_vec.data.push_back(left_vel_filtered_);
  vel_vec.data.push_back(left_vel_measured_);
  vel_vec.data.push_back(left_vel_commanded_);
  vel_vec.data.push_back(right_vel_filtered_);
  vel_vec.data.push_back(right_vel_measured_);
  vel_vec.data.push_back(right_vel_commanded_);

  vel_vec.data.push_back(motor_speeds_commanded_[LEFT_MOTOR_INDEX_]);
  vel_vec.data.push_back(motor_speeds_commanded_[RIGHT_MOTOR_INDEX_]);

  vel_calc_pub.publish(vel_vec);

  return;
}

void OpenRover::publishFastRateData()
{
  rr_openrover_driver_msgs::RawRrOpenroverDriverFastRateData msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "";

  msg.left_motor = robot_data_[i_ENCODER_INTERVAL_MOTOR_LEFT];
  msg.right_motor = robot_data_[i_ENCODER_INTERVAL_MOTOR_RIGHT];
  msg.flipper_motor = robot_data_[i_ENCODER_INTERVAL_MOTOR_FLIPPER];
  if(use_legacy_) {
    fast_rate_pub.publish(msg);
  }
  publish_fast_rate_values_ = false;
  return;
}

void OpenRover::publishMedRateData()
{
  rr_openrover_driver_msgs::RawRrOpenroverDriverMedRateData med_msg;
  std_msgs::Bool is_charging_msg;

  med_msg.header.stamp = ros::Time::now();
  med_msg.header.frame_id = "";

  med_msg.reg_pwr_total_current = robot_data_[i_REG_PWR_TOTAL_CURRENT];
  med_msg.reg_motor_fb_rpm_left = robot_data_[i_REG_MOTOR_FB_RPM_LEFT];
  med_msg.reg_motor_fb_rpm_right = robot_data_[i_REG_MOTOR_FB_RPM_RIGHT];
  med_msg.reg_flipper_fb_position_pot1 = robot_data_[i_REG_FLIPPER_FB_POSITION_POT1];
  med_msg.reg_flipper_fb_position_pot2 = robot_data_[i_REG_FLIPPER_FB_POSITION_POT2];
  med_msg.reg_motor_fb_current_left = robot_data_[i_REG_MOTOR_FB_CURRENT_LEFT];
  med_msg.reg_motor_fb_current_right = robot_data_[i_REG_MOTOR_FB_CURRENT_RIGHT];
  med_msg.reg_motor_charger_state = robot_data_[i_REG_MOTOR_CHARGER_STATE];
  med_msg.reg_power_a_current = robot_data_[i_REG_POWER_A_CURRENT];
  med_msg.reg_power_b_current = robot_data_[i_REG_POWER_B_CURRENT];
  med_msg.reg_motor_flipper_angle = robot_data_[i_REG_MOTOR_FLIPPER_ANGLE];
  med_msg.battery_current_a = robot_data_[i_BATTERY_CURRENT_A];
  med_msg.battery_current_b = robot_data_[i_BATTERY_CURRENT_B];

  if (robot_data_[i_REG_MOTOR_CHARGER_STATE] == 0xDADA)
  {
    is_charging_ = true;
    is_charging_msg.data = true;
    is_charging_pub.publish(is_charging_msg);
  }
  else
  {
    is_charging_ = false;
    is_charging_msg.data = false;
    is_charging_pub.publish(is_charging_msg);
  }

  if(use_legacy_) {
    medium_rate_pub.publish(med_msg);
  }
  publish_med_rate_values_ = false;
  return;
}

rr_openrover_driver_msgs::SmartBatteryStatus interpret_battery_status(uint16_t bits)
{
  rr_openrover_driver_msgs::SmartBatteryStatus status_msg;
  status_msg.over_charged_alarm = bool(bits & 0x8000);
  status_msg.terminate_charge_alarm = bool(bits & 0x4000);
  status_msg.over_temp_alarm = bool(bits & 0x1000);
  status_msg.terminate_discharge_alarm = bool(bits & 0x0800);
  status_msg.remaining_capacity_alarm = bool(bits & 0x0200);
  status_msg.remaining_time_alarm = bool(bits & 0x0100);
  status_msg.initialized = bool(bits & 0x0080);
  status_msg.discharging = bool(bits & 0x0040);
  status_msg.fully_charged = bool(bits & 0x0020);
  status_msg.fully_discharged = bool(bits & 0x0010);
  return status_msg;
}

void OpenRover::publishSlowRateData()
{
  rr_openrover_driver_msgs::RawRrOpenroverDriverSlowRateData slow_msg;
  rr_openrover_driver_msgs::SmartBatteryStatus batteryStatusA;

  slow_msg.header.stamp = ros::Time::now();
  slow_msg.header.frame_id = "";

  slow_msg.reg_motor_fault_flag_left = robot_data_[i_REG_MOTOR_FAULT_FLAG_LEFT];
  slow_msg.reg_motor_temp_left = robot_data_[i_REG_MOTOR_TEMP_LEFT];
  slow_msg.reg_motor_temp_right = robot_data_[i_REG_MOTOR_TEMP_RIGHT];
  slow_msg.reg_power_bat_voltage_a = robot_data_[i_REG_POWER_BAT_VOLTAGE_A];
  slow_msg.reg_power_bat_voltage_b = robot_data_[i_REG_POWER_BAT_VOLTAGE_B];
  slow_msg.reg_robot_rel_soc_a = robot_data_[i_REG_ROBOT_REL_SOC_A];
  slow_msg.reg_robot_rel_soc_b = robot_data_[i_REG_ROBOT_REL_SOC_B];
  slow_msg.battery_mode_a = robot_data_[i_BATTERY_MODE_A];
  slow_msg.battery_mode_b = robot_data_[i_BATTERY_MODE_B];
  slow_msg.battery_temp_a = robot_data_[i_BATTERY_TEMP_A];
  slow_msg.battery_temp_b = robot_data_[i_BATTERY_TEMP_B];
  slow_msg.battery_voltage_a = robot_data_[i_BATTERY_VOLTAGE_A];
  slow_msg.battery_voltage_b = robot_data_[i_BATTERY_VOLTAGE_B];
  slow_msg.buildno = robot_data_[i_BUILDNO];

  battery_status_a_pub.publish(interpret_battery_status(robot_data_[i_BATTERY_STATUS_A]));
  battery_status_b_pub.publish(interpret_battery_status(robot_data_[i_BATTERY_STATUS_B]));

  std_msgs::Int32 soc;
  soc.data = (robot_data_[i_REG_ROBOT_REL_SOC_A] + robot_data_[i_REG_ROBOT_REL_SOC_B]) / 2;
  battery_state_of_charge_pub.publish(soc);

  if(use_legacy_) {
    slow_rate_pub.publish(slow_msg);
  }
  publish_slow_rate_values_ = false;
  return;
}

void OpenRover::publishMotorSpeeds()
{
  std_msgs::Int32MultiArray motor_speeds_msg;
  motor_speeds_msg.data.clear();
  motor_speeds_msg.data.push_back(motor_speeds_commanded_[LEFT_MOTOR_INDEX_]);
  motor_speeds_msg.data.push_back(motor_speeds_commanded_[RIGHT_MOTOR_INDEX_]);
  motor_speeds_msg.data.push_back(motor_speeds_commanded_[FLIPPER_MOTOR_INDEX_]);

  motor_speeds_pub.publish(motor_speeds_msg);
  return;
}

void OpenRover::serialManager()
{  // sends serial commands stored in the 3 buffers in order of speed with fast getting highest priority
  unsigned char param1;
  unsigned char param2;
  static double past_time = 0;

  while ((serial_fast_buffer_.size() > 0) || (serial_medium_buffer_.size() > 0) || (serial_slow_buffer_.size() > 0) ||
         (serial_fan_buffer_.size() > 0))
  {
    // Fast data gets highest priority from being first in this if statement
    // If the CPU running the driver can only process 60 commands / second and the fast
    // data rate is set to 60hz, no other data will be gathered and the medium and slow Buffers
    // will fill up and issue a warning.
    if (serial_fast_buffer_.size() > 0)
    {
      param1 = 10;
      param2 = serial_fast_buffer_.back();
      serial_fast_buffer_.pop_back();
      ROS_DEBUG("Its fast data's turn to be sent: %i", param2);
    }
    else if (serial_fan_buffer_.size() > 0)
    {
      param1 = 20;
      param2 = serial_fan_buffer_.back();
      serial_fan_buffer_.pop_back();
      ROS_DEBUG("Its fan speed's turn to be sent: %i", param2);
    }
    else if (serial_medium_buffer_.size() > 0)
    {
      param1 = 10;
      param2 = serial_medium_buffer_.back();
      serial_medium_buffer_.pop_back();
      ROS_DEBUG("Its medium data's turn to be sent: %i", param2);
    }
    else if (serial_slow_buffer_.size() > 0)
    {
      param1 = 10;
      param2 = serial_slow_buffer_.back();
      serial_slow_buffer_.pop_back();
      ROS_DEBUG("Its slow data's turn to be sent: %i", param2);
    }
    else
    {
      param2 = 0;
      param1 = 0;
    }

    // Check param1 to determine what communication to the robot is required
    try
    {
      if (param1 == 10)  // Param1==10 requests data with index of param2
      {
        updateRobotData(param2);
      }
      else if (param1 == 20)
      {  // param1==20 means sending fan speed of param2
        setParameterData(param1, param2);
      }
      else if (param1 == 250)
      {  // param1==250 means calibrating flipper DO NOT USE OFTEN
        setParameterData(param1, param2);
      }
      else if (param1 == 0)
      {  // param1==0 means buffers are empty and doesn't need to do anything
      }
      else
      {
        throw std::string("Unknown param1. Removing parameter from buffer");
      }
    }
    catch (std::string s)
    {
      throw;
    }
    catch (...)
    {
      throw;
    }

    // If one of the buffers are empty, publish the values
    if ((serial_fast_buffer_.size() == 0) && publish_fast_rate_values_)
    {
      ros::Time ros_now_time = ros::Time::now();
      double now_time = ros_now_time.toSec();

      double dt = now_time - past_time;
      past_time = now_time;
      publishFastRateData();
      updateMeasuredVelocities();  // Update openrover measured velocities based on latest encoder readings

      motor_speeds_commanded_[LEFT_MOTOR_INDEX_] =
          left_controller_.run(e_stop_on_, closed_loop_control_on_, left_vel_commanded_, left_vel_measured_, dt);
      motor_speeds_commanded_[RIGHT_MOTOR_INDEX_] =
          right_controller_.run(e_stop_on_, closed_loop_control_on_, right_vel_commanded_, right_vel_measured_, dt);

      left_vel_filtered_ = left_controller_.velocity_filtered_;
      right_vel_filtered_ = right_controller_.velocity_filtered_;

      publishOdometry(left_vel_measured_, right_vel_measured_);  // Publish new calculated odometry
      publishWheelVels();                                        // call after publishOdometry()
      publishMotorSpeeds();
    }
    else if ((serial_medium_buffer_.size() == 0) && publish_med_rate_values_)
    {
      publishMedRateData();
    }
    else if ((serial_slow_buffer_.size() == 0) && publish_slow_rate_values_)
    {
      publishSlowRateData();
    }

    // Checks timers and subscribers
    ros::spinOnce();
  }

  if ((serial_fast_buffer_.size() == 0) && publish_fast_rate_values_)
  {
    publish_fast_rate_values_ = false;
  }

  if ((serial_medium_buffer_.size() == 0) && publish_med_rate_values_)
  {
    publish_med_rate_values_ = false;
  }

  if ((serial_slow_buffer_.size() == 0) && publish_slow_rate_values_)
  {
    publish_slow_rate_values_ = false;
  }

  return;
}

void OpenRover::updateMeasuredVelocities()
{
  int left_enc = robot_data_[i_ENCODER_INTERVAL_MOTOR_LEFT];
  int right_enc = robot_data_[i_ENCODER_INTERVAL_MOTOR_RIGHT];

  // Bound left_encoder readings to range of normal operation.
  if (left_enc < ENCODER_MIN)
  {
    left_vel_measured_ = 0;
  }
  else if (left_enc > ENCODER_MAX)
  {
    left_vel_measured_ = 0;
  }
  else if (motor_speeds_commanded_[LEFT_MOTOR_INDEX_] > MOTOR_NEUTRAL)  // this sets direction of measured
  {
    left_vel_measured_ = odom_encoder_coef_ / left_enc;
  }
  else
  {
    left_vel_measured_ = -odom_encoder_coef_ / left_enc;
  }

  // Bound right_encoder readings to range of normal operation.
  if (right_enc < ENCODER_MIN)
  {
    right_vel_measured_ = 0;
  }
  else if (right_enc > ENCODER_MAX)
  {
    right_vel_measured_ = 0;
  }
  else if (motor_speeds_commanded_[RIGHT_MOTOR_INDEX_] > MOTOR_NEUTRAL)  // this sets direction of measured
  {
    right_vel_measured_ = odom_encoder_coef_ / right_enc;
  }
  else
  {
    right_vel_measured_ = -odom_encoder_coef_ / right_enc;
  }

  return;
}

void OpenRover::updateRobotData(int param)
{
  try
  {
    int data = getParameterData(param);
    if (data < 0)  // check if val is good (not negative) and if not, push param back to buffer
    {
      throw;
    }

    robot_data_[param] = data;
  }
  catch (std::string s)
  {
    char str_ex[50];
    sprintf(str_ex, "Failed to update param %i. ", param);
    throw std::string(str_ex) + s;
  }
  return;
}

bool OpenRover::sendCommand(int param1, int param2)
{
  unsigned char write_buffer[SERIAL_OUT_PACKAGE_LENGTH];

  write_buffer[0] = SERIAL_START_BYTE;
  write_buffer[1] = (unsigned char)motor_speeds_commanded_[LEFT_MOTOR_INDEX_];     // left motor
  write_buffer[2] = (unsigned char)motor_speeds_commanded_[RIGHT_MOTOR_INDEX_];    // right motor
  write_buffer[3] = (unsigned char)motor_speeds_commanded_[FLIPPER_MOTOR_INDEX_];  // flipper
  write_buffer[4] = (unsigned char)param1;  // Param 1: 10 to get data, 240 for low speed mode
  write_buffer[5] = (unsigned char)param2;  // Param 2:
  // Calculate Checksum
  write_buffer[6] =
      (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3] + write_buffer[4] + write_buffer[5]) % 255;

  if (!(serial_port_fd_ > 0))
  {
    ROS_INFO("Serial communication failed. Attempting to restart.");
    if (!(openComs()))
    {
      ROS_WARN("Failed to restart serial communication.");
    }
  }

  if (write(serial_port_fd_, write_buffer, SERIAL_OUT_PACKAGE_LENGTH) < SERIAL_OUT_PACKAGE_LENGTH)
  {
    char str_ex[50];
    sprintf(str_ex, "Failed to send command: %02x,%02x,%02x,%02x,%02x,%02x,%02x", write_buffer[0], write_buffer[1],
            write_buffer[2], write_buffer[3], write_buffer[4], write_buffer[5], write_buffer[6]);
    throw std::string(str_ex);
  }

  return true;
}

int OpenRover::readCommand()
{  // only used after a send command with param1==10
  unsigned char read_buffer[1];
  unsigned char start_byte_read, checksum, read_checksum, data1, data2, dataNO;
  int data;
  if (!(serial_port_fd_ > 0))
  {
    ROS_INFO("Serial communication failed. Attempting to restart.");
    if (!(openComs()))
    {
      ROS_WARN("Failed to restart serial communication.");
    }
  }

  int bits_read = read(serial_port_fd_, read_buffer, 1);
  start_byte_read = read_buffer[0];

  read(serial_port_fd_, read_buffer, 1);  // get param
  dataNO = read_buffer[0];

  read(serial_port_fd_, read_buffer, 1);  // get data1
  data1 = read_buffer[0];

  read(serial_port_fd_, read_buffer, 1);  // get data2
  data2 = read_buffer[0];

  read(serial_port_fd_, read_buffer, 1);  // get checksum
  read_checksum = read_buffer[0];

  checksum = 255 - (dataNO + data1 + data2) % 255;

  if (!(SERIAL_START_BYTE == start_byte_read))
  {
    char str_ex[50];
    sprintf(str_ex, "Received bad start byte. Received: %02x", start_byte_read);
    tcflush(serial_port_fd_, TCIOFLUSH);  // flush received buffer
    throw std::string(str_ex);
  }
  else if (checksum != read_checksum)
  {
    char str_ex[50];
    sprintf(str_ex, "Received bad CRC. Received: %02x,%02x,%02x,%02x,%02x", start_byte_read, dataNO, data1, data2,
            read_checksum);
    tcflush(serial_port_fd_, TCIOFLUSH);  // flush received buffer
    throw std::string(str_ex);
  }
  data = (data1 << 8) + data2;
  return data;
}

bool OpenRover::setParameterData(int param1, int param2)
{
  try
  {
    if (!sendCommand(param1, param2))
    {
      throw;
    }

    return true;
  }
  catch (std::string s)
  {
    std::string s2("setParameterData() failed. ");
    throw(s2 + s);
  }
}

int OpenRover::getParameterData(int param)
{
  int data;

  try
  {
    if (!sendCommand(10, param))
    {
      throw;
    }

    data = readCommand();

    if (0 > data)
    {
      throw;
    }
    return data;
  }
  catch (std::string s)
  {
    std::string s2("getParameterData() failed. ");  // %i. ", param);
    throw(s2 + s);
  }
}

bool OpenRover::openComs()
{
  ROS_INFO("Opening serial port");
  struct termios serial_port_fd__options;

  serial_port_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_port_fd_ < 0)
  {
    ROS_ERROR("Failed to open port: %s", strerror(errno));
    return false;
  }
  if (0 > fcntl(serial_port_fd_, F_SETFL, 0))
  {
    ROS_ERROR("Failed to set port descriptor: %s", strerror(errno));
    return false;
  }
  if (0 > tcgetattr(serial_port_fd_, &serial_port_fd__options))
  {
    ROS_ERROR("Failed to fetch port attributes: %s", strerror(errno));
    return false;
  }
  if (0 > cfsetispeed(&serial_port_fd__options, B57600))
  {
    ROS_ERROR("Failed to set input baud: %s", strerror(errno));
    return false;
  }
  if (0 > cfsetospeed(&serial_port_fd__options, B57600))
  {
    ROS_ERROR("Failed to set output baud: %s", strerror(errno));
    return false;
  }

  serial_port_fd__options.c_cflag |= (CREAD | CLOCAL | CS8);
  serial_port_fd__options.c_cflag &= ~(PARODD | CRTSCTS | CSTOPB | PARENB);
  serial_port_fd__options.c_iflag &= ~(IUCLC | IXANY | IMAXBEL | IXON | IXOFF | IUTF8 | ICRNL | INPCK);  // input modes
  serial_port_fd__options.c_oflag |= (NL0 | CR0 | TAB0 | BS0 | VT0 | FF0);
  serial_port_fd__options.c_oflag &=
      ~(OPOST | ONLCR | OLCUC | OCRNL | ONOCR | ONLRET | OFILL | OFDEL | NL1 | CR1 | CR2 | TAB3 | BS1 | VT1 | FF1);
  serial_port_fd__options.c_lflag |= (NOFLSH);
  serial_port_fd__options.c_lflag &= ~(ICANON | IEXTEN | TOSTOP | ISIG | ECHOPRT | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
  serial_port_fd__options.c_cc[VINTR] = 0x03;   // INTR Character
  serial_port_fd__options.c_cc[VQUIT] = 0x1C;   // QUIT Character
  serial_port_fd__options.c_cc[VERASE] = 0x7F;  // ERASE Character
  serial_port_fd__options.c_cc[VKILL] = 0x15;   // KILL Character
  serial_port_fd__options.c_cc[VEOF] = 0x04;    // EOF Character
  serial_port_fd__options.c_cc[VTIME] = 0x01;   // Timeout in 0.1s of serial read
  serial_port_fd__options.c_cc[VMIN] = 0;       // SERIAL_IN_PACKAGE_LENGTH; //Min Number of bytes to read
  serial_port_fd__options.c_cc[VSWTC] = 0x00;
  serial_port_fd__options.c_cc[VSTART] = SERIAL_START_BYTE;  // START Character
  serial_port_fd__options.c_cc[VSTOP] = 0x13;                // STOP character
  serial_port_fd__options.c_cc[VSUSP] = 0x1A;                // SUSP character
  serial_port_fd__options.c_cc[VEOL] = 0x00;                 // EOL Character
  serial_port_fd__options.c_cc[VREPRINT] = 0x12;
  serial_port_fd__options.c_cc[VDISCARD] = 0x0F;
  serial_port_fd__options.c_cc[VWERASE] = 0x17;
  serial_port_fd__options.c_cc[VLNEXT] = 0x16;
  serial_port_fd__options.c_cc[VEOL2] = 0x00;

  if (0 > tcsetattr(serial_port_fd_, TCSANOW, &serial_port_fd__options))
  {
    ROS_ERROR("Failed to set port attributes: %s", strerror(errno));
    return false;
  }
  ::ioctl(serial_port_fd_, TIOCEXCL);  // turn on exclusive mode

  ROS_INFO("Serial port opened");
  is_serial_coms_open_ = true;
  tcflush(serial_port_fd_, TCIOFLUSH);  // flush received buffer

  return true;
}

}  // namespace openrover

int main(int argc, char* argv[])
{
  // Create ROS node
  ros::init(argc, argv, "rr_openrover_driver_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_priv("~");
  openrover::OpenRover openrover(nh, nh_priv);
  /*        if( !nh )
          {
                  ROS_FATAL( "Failed to initialize NodeHandle" );
                  ros::shutdown( );
                  return -1;
          }
          if( !nh_priv )
          {
                  ROS_FATAL( "Failed to initialize private NodeHandle" );
                  delete nh;
                  ros::shutdown( );
                  return -2;
          }
          if( !openrover )
          {
                  ROS_FATAL( "Failed to initialize driver" );
                  delete nh_priv;
                  delete nh;
                  ros::shutdown( );
                  return -3;
          }
  */
  if (!openrover.start())
  {
    ROS_FATAL("Failed to start the openrover driver");
    ros::requestShutdown();
  }

  ros::Rate loop_rate(openrover::LOOP_RATE);

  while (ros::ok())
  {
    try
    {
      ros::spinOnce();
      // Process Serial Buffers
      openrover.serialManager();
      loop_rate.sleep();  // sleeping greatly reduces CPU
    }
    catch (std::string s)
    {
      ROS_ERROR("%s", s.c_str());
    }
    catch (...)
    {
      ROS_ERROR("Unknown Exception occurred");
    }
  }
  /*
          delete openrover;
          delete nh_priv;
          delete nh;
  */

  return 0;
}
