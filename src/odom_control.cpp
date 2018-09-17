#include <ctime>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include "ros/ros.h"

#include <rr_openrover_basic/odom_control.hpp>

namespace openrover {

OdomControl::OdomControl(bool use_control, double Kp, double Ki, 
        double Kd, char max, char min, std::string log_filename) :

    MOTOR_NEUTRAL_(125),
    MOTOR_MAX_(250),
    MOTOR_MIN_(0),
    MOTOR_DEADBAND_(9),
    MAX_ACCEL_CUTOFF_(20.0),
    MIN_VELOCITY_(0.03),
    MAX_VELOCITY_(3),
    enable_file_logging_(false),
    log_filename(log_filename),
    K_P_(Kp),
    K_I_(Ki),
    K_D_(Kd),
    velocity_history_(3, 0),
    use_control_(use_control),
    skip_measurement_(false),
    at_max_motor_speed_(false),
    at_min_motor_speed_(false)
{
}

OdomControl::OdomControl(bool use_control, double Kp, double Ki, 
        double Kd, char max, char min) :

    MOTOR_NEUTRAL_(125),
    MOTOR_MAX_(250),
    MOTOR_MIN_(0),
    MOTOR_DEADBAND_(9),
    MAX_ACCEL_CUTOFF_(20.0),
    MIN_VELOCITY_(0.03),
    MAX_VELOCITY_(3),
    enable_file_logging_(false),
    K_P_(Kp),
    K_I_(Ki),
    K_D_(Kd),
    velocity_history_(3, 0),
    use_control_(use_control),
    skip_measurement_(false),
    at_max_motor_speed_(false),
    at_min_motor_speed_(false),
    error_(0),
    integral_value_(0)
{
}

char OdomControl::calculate(double commanded_vel, double measured_vel, double dt)
{
    velocity_filtered_ = filter(measured_vel, dt);
    error_ = commanded_vel - velocity_filtered_;
    ROS_INFO("error_ = %3.3f", error_);
    if (!skip_measurement_) 
    {
        ROS_INFO("Running PID loop");
        motor_speed_ = PID(error_, dt);
    }

    if (hasZeroHistory(velocity_history_))
    {
        ROS_INFO("Has zero history");
        motor_speed_ = MOTOR_NEUTRAL_;
    }

    motor_speed_ = deadbandOffset(motor_speed_, MOTOR_DEADBAND_);
    motor_speed_ = boundMotorSpeed(motor_speed_, MOTOR_MAX_, MOTOR_MIN_); 

    return motor_speed_;
}

char OdomControl::PID(double error, double dt)
{
    double p_val = P(error, dt);
    double i_val = I(error, dt);
    double d_val = D(error, dt);

    ROS_INFO("%4.4f | %4.4f | %4.4f", p_val, i_val, d_val);
    return char(p_val + i_val + d_val);
}

double OdomControl::D(double error, double dt)
{
    return K_D_ * (velocity_history_[0]-velocity_history_[1]) / dt;
}

double OdomControl::I(double error, double dt)
{
    integral_value_ += (K_I_*error)*dt;
    return integral_value_;
}

double OdomControl::P(double error, double dt)
{
    return error*K_P_;
}

bool OdomControl::hasZeroHistory(const std::vector<double>& vel_history)
{
    float sum = fabs(vel_history[0] + vel_history[1] + vel_history[2]);
    if (sum < 0.001)
        return true;
    else
        return false;
}

int OdomControl::boundMotorSpeed(int motor_speed, int max, int min)
{
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

    return motor_speed;
}

char OdomControl::deadbandOffset(char motor_speed, char deadband_offset)
{
    //Compensate for deadband 
    if (motor_speed > 125) 
    { 
        motor_speed += deadband_offset; 
    } 
    else if (motor_speed < 125 ) 
    { 
        motor_speed -= deadband_offset; 
    } 
}

double OdomControl::filter(double velocity, double dt)
{
    static double time = 0;

    if (skip_measurement_)
    {
        time += dt;
    }
    else
    {
        time = dt;
    }

    //Check for impossible acceleration
    float accel = (velocity - velocity_history_[0]) / time;

    if (fabs(velocity) > MAX_ACCEL_CUTOFF_)
    {
        skip_measurement_ = true;
        throw std::string("Skipping left encoder reading");
    }
    else
    {
        skip_measurement_ = false;
        //Hanning filter
        velocity_filtered_ = 0.25 * velocity + 0.5 * velocity_history_[0] + 0.25 * velocity_history_[1];
        velocity_history_.insert(velocity_history_.begin(), velocity_filtered_);
        velocity_history_.pop_back();
    }
    return velocity_filtered_;

    //for billinear transform
/*    static std::vector<float>  right_x_history(3,0);
    static std::vector<float>  left_x_history(3,0);
    left_x_history.insert(left_x_history.begin(), left_vel);
    left_x_history.pop_back();
*/

    //dumb low pass filter
/*    velocity_filtered_ = velocity_measured_ / 2 + velocity_filtered_ / 2;


    //Billinear 2nd Order IIR Butterworth Filter
    //x_history is measured velocitys
/*    float a1 = 0.20657;
    float a2 = 0.41314;
    float a3 = 0.20657;
    float b1 = 0.36953;
    float b2 = -0.19582;

    velocity_filtered_ = a1*left_x_history[0] + a2*left_x_history[1] + a3*left_x_history[2] + 
        b1*velocity_history_[0] + b2*velocity_history_[1];
    velocity_history_.insert(velocity_history_.begin(), velocity_filtered_);
    velocity_history_.pop_back();
    //ROS_INFO("%1.3f ||| %1.3f %1.3f %1.3f", velocity_filtered_, left_vel, velocity_history_[0], velocity_history_[1]);
 */
}

}