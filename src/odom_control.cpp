#include <ctime>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>

#include <rr_openrover_basic/odom_control.h>

namespace openrover {

OdomControl::OdomControl(bool use_control, double Kp, double Ki, double Kd, char max, char min, std::string log_filename) :
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
    use_control_(use_control)
{
}

char OdomControl::calculate(double commanded_vel, double measured_vel, double dt)
{
    filter(measured_vel, dt);

    if (hasZeroHistory(vel_history))
    {
        motor_speed_ = MOTOR_NEUTRAL_;
    }
    return motor_speed_
}

bool OdomControl::hasZeroHistory(const std::vector<float>& vel_history)
{
    float sum = fabs(vel_history[0] + vel_history[1] + vel_history[2]);
    //ROS_INFO("vel_history ||| %3.3f %3.3f %3.3f | %3.3f", vel_history[0], vel_history[1], vel_history[3], sum);
    if (sum < 0.001)
        return true;
    else
        return false;
}

int OdomControl::boundMotorSpeed(int motor_speed, int max, int min)
{
    if (motor_speed > max)
    {
        motor_speed = max;
        ROS_WARN("Reached full forward motor speed.");
    }
    if (motor_speed < min)
    {
        motor_speed = min;
        ROS_WARN("Reached full reverse motor speed.");
    }

    return motor_speed;
}


void OdomControl::filter(float velocity, float dt)
{
    static double time = 0;

    if (skip_velocity_)
    {
        time += dt;
    }
    else
    {
        time = dt;
    }

    //Check for impossible acceleration
    float accel = (velocity - velocity_history_[0]) / time;
/*    ROS_INFO("%1.4f | %1.4f", time, right_time);
    ROS_INFO("%3.4f | %3.4f", left_accel, right_accel);
*/
    if (fabs(velocity) > MAX_ACCEL_CUTOFF_)
    {
        throw std::string("Skipping left encoder reading");
        skip_velocity_ = true;
    }
    else
    {
        skip_velocity_ = false;
        //Hanning filter
        velocity_filtered_ = 0.25 * left_vel + 0.5 * velocity_history_[0] + 0.25 * velocity_history_[1];
        velocity_history_.insert(velocity_history_.begin(), velocity_filtered_);
        velocity_history_.pop_back();
    }

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