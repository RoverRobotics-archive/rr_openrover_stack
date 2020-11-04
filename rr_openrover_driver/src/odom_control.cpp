#include <ctime>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include "ros/ros.h"

#include <rr_openrover_driver/odom_control.hpp>
#include <rr_openrover_driver/constants.hpp>

namespace openrover
{
void OdomControl::start(bool use_control, PidGains pid_gains, int max, int min)
{
  K_P_ = pid_gains.Kp;
  K_I_ = pid_gains.Ki;
  K_D_ = pid_gains.Kd;
}

OdomControl::OdomControl()
  : MOTOR_MAX_(MOTOR_SPEED_MAX)
  , MOTOR_MIN_(MOTOR_SPEED_MIN)
  , MOTOR_DEADBAND_(9)
  , MAX_ACCEL_CUTOFF_(5.0)
  , MIN_VELOCITY_(0.03)
  , MAX_VELOCITY_(5)
  , fs_(nullptr)
  , K_P_(0)
  , K_I_(0)
  , K_D_(0)
  , velocity_filtered_history_(5, 0)
  , velocity_history_(3, 0)
  , use_control_(false)
  , skip_measurement_(false)
  , at_max_motor_speed_(false)
  , at_min_motor_speed_(false)
  , stop_integrating_(false)
  , velocity_error_(0)
  , integral_error_(0)
  , differential_error_(0)
  , velocity_commanded_(0)
  , velocity_measured_(0)
  , velocity_filtered_(0)
{
  ROS_INFO("odom Kp: %f", K_P_);
  ROS_INFO("odom Ki: %f", K_I_);
  ROS_INFO("odom Kd: %f", K_D_);
}

OdomControl::OdomControl(bool use_control, PidGains pid_gains, int max, int min, std::ofstream* fs)
  : MOTOR_MAX_(max)
  , MOTOR_MIN_(min)
  , MOTOR_DEADBAND_(9)
  , MAX_ACCEL_CUTOFF_(5.0)
  , MIN_VELOCITY_(0.03)
  , MAX_VELOCITY_(3)
  , fs_(fs)
  , K_P_(pid_gains.Kp)
  , K_I_(pid_gains.Ki)
  , K_D_(pid_gains.Kd)
  , velocity_filtered_history_(5, 0)
  , velocity_history_(3, 0)
  , use_control_(use_control)
  , skip_measurement_(false)
  , at_max_motor_speed_(false)
  , at_min_motor_speed_(false)
  , stop_integrating_(false)
  , velocity_error_(0)
  , integral_error_(0)
  , differential_error_(0)
  , velocity_commanded_(0)
  , velocity_measured_(0)
  , velocity_filtered_(0)
{
  ROS_INFO("odom Kp: %f", K_P_);
  ROS_INFO("odom Ki: %f", K_I_);
  ROS_INFO("odom Kd: %f", K_D_);

  if (fs_ != nullptr && fs_->is_open()) {
    *fs_ << "time,Kp,Ki,Kd,error,integral_error,differential_error,error_filtered,meas_vel,filt_vel,cmd_vel,dt,motor_cmd\n";
    fs_->flush();
  }
}

OdomControl::OdomControl(bool use_control, PidGains pid_gains, int max, int min)
  : MOTOR_MAX_(max)
  , MOTOR_MIN_(min)
  , MOTOR_DEADBAND_(9)
  , MAX_ACCEL_CUTOFF_(5.0)
  , MIN_VELOCITY_(0.03)
  , MAX_VELOCITY_(3)
  , fs_(nullptr)
  , K_P_(pid_gains.Kp)
  , K_I_(pid_gains.Ki)
  , K_D_(pid_gains.Kd)
  , velocity_filtered_history_(5, 0)
  , velocity_history_(3, 0)
  , use_control_(use_control)
  , skip_measurement_(false)
  , at_max_motor_speed_(false)
  , at_min_motor_speed_(false)
  , stop_integrating_(false)
  , velocity_error_(0)
  , integral_error_(0)
  , differential_error_(0)
  , velocity_commanded_(0)
  , velocity_measured_(0)
  , velocity_filtered_(0)
{
  ROS_INFO("odom Kp: %f", K_P_);
  ROS_INFO("odom Ki: %f", K_I_);
  ROS_INFO("odom Kd: %f", K_D_);
}

unsigned char OdomControl::run(bool e_stop_on, bool control_on, double commanded_vel, double measured_vel, double dt)
{
  velocity_commanded_ = commanded_vel;

  velocity_measured_ = measured_vel;

  velocity_filtered_ = filter(measured_vel, dt);

  std::cout << "velocity filtered: " << velocity_filtered_ << std::endl;
  std::cout << "dt: " << dt << std::endl;


  //If rover is E-Stopped, respond with NEUTRAL comman
  if (e_stop_on)
  {
    reset();
    return MOTOR_NEUTRAL;
  }

  // If stopping, stop now when velocity has slowed.
  if ((commanded_vel == 0.0) && (fabs(velocity_filtered_)<0.3))
  {
    integral_error_ = 0;
    if (hasZeroHistory(velocity_filtered_history_))
    {
      return MOTOR_NEUTRAL;
    }
  }

  // If controller should be ON, run it.
  if (control_on)
  {
    velocity_error_ = commanded_vel - velocity_filtered_;
    if (!skip_measurement_)
    {
      motor_speed_ = PID(velocity_error_, dt);
      ROS_DEBUG("PID: %i", motor_speed_);
    }
  }
  else
  {
    motor_speed_ = feedThroughControl();
  }

  motor_speed_ = boundMotorSpeed(motor_speed_, MOTOR_MAX_, MOTOR_MIN_);

  if (fs_ != nullptr && fs_->is_open()){
    *fs_ << ros::Time::now() << ",";
    *fs_ << K_P_ << "," << K_I_ << "," << K_D_ << ",";
    *fs_ << commanded_vel - measured_vel << "," << integral_error_ << "," << differential_error_<< "," << velocity_error_ << ",";
    *fs_ << measured_vel << "," << velocity_filtered_ << "," << commanded_vel << ",";
    *fs_ << dt << "," << motor_speed_ << "\n";
    fs_->flush();
  }

  return (unsigned char)motor_speed_;
}

int OdomControl::feedThroughControl()
{
  return (int)round(velocity_commanded_ * 50 + MOTOR_NEUTRAL);
}

void OdomControl::reset()
{
  integral_error_ = 0;
  velocity_error_ = 0;
  velocity_commanded_ = 0;
  velocity_measured_ = 0;
  velocity_filtered_ = 0;
  std::fill(velocity_filtered_history_.begin(), velocity_filtered_history_.end(), 0);
  motor_speed_ = MOTOR_NEUTRAL;
  skip_measurement_ = false;
}

int OdomControl::PID(double error, double dt)
{
  double p_val = P(error, dt);
  double i_val = I(error, dt);
  double d_val = D(error, dt);
  double pid_val = p_val + i_val + d_val;
  ROS_DEBUG("\nerror: %lf\n dt: %lf", error, dt);
  ROS_DEBUG("\n kp: %lf \n ki: %lf \n kd: %lf \n", p_val, i_val, d_val);

  if (fabs(pid_val) > (MOTOR_MAX_/2.0))
    //Only integrate if the motor's aren't already at full speed
  {
    stop_integrating_ = true;
  }
  else
  {
    stop_integrating_ = false;
  }

  return (int)round(pid_val + 125.0);
}

double OdomControl::D(double error, double dt)
{
  differential_error_ = (velocity_filtered_history_[0] - velocity_filtered_history_[1]) / dt;
  return K_D_ * differential_error_;
}

double OdomControl::I(double error, double dt)
{
  if (!stop_integrating_)
  {
    integral_error_ += error * dt;
  }
  return K_I_ * integral_error_;
}

double OdomControl::P(double error, double dt)
{
  double p_val = error * K_P_;
  return error * K_P_;
}

bool OdomControl::hasZeroHistory(const std::vector<double>& vel_history)
{
  double avg = (fabs(vel_history[0]) + fabs(vel_history[1]) + fabs(vel_history[2])) / 3.0;
  if (avg < 0.03)
    return true;
  else
    return false;
}

int OdomControl::boundMotorSpeed(int motor_speed, int max, int min)
{
  int test_motor;
  int test_factor = 18;
  at_max_motor_speed_ = false;
  at_min_motor_speed_ = false;

  if (motor_speed > max)
  {
    motor_speed = max;
    at_max_motor_speed_ = true;
  }
  if (motor_speed < min)
  {
    motor_speed = min;
    at_min_motor_speed_ = true;
  }

  test_motor = motor_speed;

  return test_motor;
}

int OdomControl::deadbandOffset(int motor_speed, int deadband_offset)
{
  // Compensate for deadband
  if (motor_speed > MOTOR_NEUTRAL)
  {
    return (motor_speed + deadband_offset);
  }
  else if (motor_speed < MOTOR_NEUTRAL)
  {
    return (motor_speed - deadband_offset);
  }
}

double OdomControl::filter(double velocity, double dt)
{
  static double time = 0;
  float change_in_velocity = 0;

  // Check for impossible acceleration, if it is impossible, ignore the measurement.
  float accel = (velocity - velocity_filtered_history_[0]) / dt;
  velocity_history_.insert(velocity_history_.begin(), velocity);
  velocity_history_.pop_back();

  /*
  if (accel > MAX_ACCEL_CUTOFF_)
  {
    change_in_velocity = 0.5*dt*MAX_ACCEL_CUTOFF_;
    velocity = velocity_filtered_history_[0] + change_in_velocity;
  }
  else if (accel < -MAX_ACCEL_CUTOFF_)
  {
    change_in_velocity = -0.5*dt*MAX_ACCEL_CUTOFF_;
    velocity = velocity_filtered_history_[0] + change_in_velocity;
  }*/

  // Hanning low pass filter filter
  // velocity_filtered_ = 0.25 * velocity + 0.5 * velocity_filtered_history_[0] + 0.25 * velocity_filtered_history_[1];
  //velocity_filtered_ = 0.1 * velocity + 0.25 * velocity_filtered_history_[0] + 0.30 * velocity_filtered_history_[1] + 0.25 * velocity_filtered_history_[2] + 0.1 * velocity_filtered_history_[3];
  velocity_filtered_ = 0.3 * velocity + 0.7 * velocity_filtered_history_[0]; //+ 0.30 * velocity_filtered_history_[1] + 0.25 * velocity_filtered_history_[2] + 0.1 * velocity_filtered_history_[3];
  velocity_filtered_history_.insert(velocity_filtered_history_.begin(), velocity_filtered_);
  velocity_filtered_history_.pop_back();

  return velocity_filtered_;
}


}  // namespace openrover
