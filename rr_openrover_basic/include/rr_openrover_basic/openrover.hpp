#ifndef _openrover_hpp
#define _openrover_hpp

#include <ros/ros.h>
#include <ros/timer.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <stdint.h>
#include <string>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "rr_openrover_basic/RawRrOpenroverBasicFastRateData.h"
#include "rr_openrover_basic/RawRrOpenroverBasicMedRateData.h"
#include "rr_openrover_basic/RawRrOpenroverBasicSlowRateData.h"
#include "rr_openrover_basic/SmartBatteryStatus.h"


#include <rr_openrover_basic/odom_control.hpp>

namespace openrover {

class OpenRover
{
public:
    OpenRover( ros::NodeHandle& nh, ros::NodeHandle& nh_priv );
    /*OpenRover(std::string port_, std::string drive_type_, bool enable_timeout_, float total_weight_,
        float slippage_factor_, float odom_covariance_0_, float odom_covariance_35_);*/
    OdomControl left_controller_; //(bool use_control, double Kp, double Ki, double Kd, unsigned char max, unsigned char min);
    OdomControl right_controller_; //(bool use_control, double Kp, double Ki, double Kd, unsigned char max, unsigned char min);

    bool start();
    bool openComs();
    bool setupRobotParams();
    void updateOdometry();

    void robotDataFastCB( const ros::WallTimerEvent &e);
    void robotDataMediumCB( const ros::WallTimerEvent &e);
    void robotDataSlowCB( const ros::WallTimerEvent &e);
    void timeoutCB( const ros::WallTimerEvent &e );

    void serialManager();

    bool publish_fast_rate_vals_;
    bool publish_med_rate_vals_;
    bool publish_slow_rate_vals_;
    bool closed_loop_control_on_;
    bool e_stop_on_;

private:
    //.csv Debugging
    std::ofstream fs_;

    //ROS Parameters
    std::string port_;
    std::string drive_type_;

    float timeout_; //Default to neutral motor values after timeout seconds

    //ROS node handlers
    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_priv_;

    //ROS Timers
    ros::WallTimer fast_timer;
    ros::WallTimer medium_timer;
    ros::WallTimer slow_timer;
    ros::WallTimer timeout_timer;

    //ROS Publisher and Subscribers
    ros::Publisher odom_enc_pub;
    ros::Publisher battery_state_pub;
    ros::Publisher is_charging_pub;
    ros::Publisher motor_speeds_pub;
    ros::Publisher vel_calc_pub;

    ros::Publisher fast_rate_pub;
    ros::Publisher medium_rate_pub;
    ros::Publisher slow_rate_pub;
    ros::Publisher battery_status_a_pub, battery_status_b_pub;

    ros::Subscriber cmd_vel_sub;
    ros::Subscriber fan_speed_sub;
    ros::Subscriber e_stop_sub;
    ros::Subscriber e_stop_reset_sub;

    //General Class variables
    int baud_; //serial baud rate
    int fd;
    int robot_data_[70]; //stores all received data from robot
    int is_charging_;
    int motor_speeds_commanded_[3]; //stores most recent commanded motor speeds
    const int LEFT_MOTOR_INDEX_;
    const int RIGHT_MOTOR_INDEX_;
    const int FLIPPER_MOTOR_INDEX_;
    bool is_serial_coms_open_;
    double fast_rate_; //update rate for encoders, 10Hz recommended
    double medium_rate_;
    double slow_rate_;


    //drive dependent parameters
    float odom_encoder_coef_;
    float odom_axle_track_;
    float odom_angular_coef_;
    float odom_traction_factor_;
    float odom_covariance_0_;
    float odom_covariance_35_;

    //velocity feedback
    double left_vel_commanded_;
    double right_vel_commanded_;
    double left_vel_measured_;
    double right_vel_measured_;
    double left_vel_filtered_;
    double right_vel_filtered_;
    double K_P_;
    double K_I_;
    double K_D_;

    int motor_speed_linear_coef_;
    int motor_speed_angular_coef_;
    int motor_speed_flipper_coef_;
    int motor_speed_deadband_;
    int motor_speed_angular_deadband_;
    float weight_coef_; //(weight_coef_>1)
    float cw_turn_coef_;

    float total_weight_; //in kg
    //int motor_speed_diff_max_; ---WIP
    geometry_msgs::Twist cmd_vel_commanded_;

    std::vector<unsigned char> serial_fast_buffer_;
    std::vector<unsigned char> serial_medium_buffer_;
    std::vector<unsigned char> serial_slow_buffer_;
    std::vector<unsigned char> serial_fan_buffer_;

    //ROS Subscriber callback functions
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
    void fanSpeedCB(const std_msgs::Int32::ConstPtr& msg);
    void eStopCB(const std_msgs::Bool::ConstPtr& msg);
    void eStopResetCB(const std_msgs::Bool::ConstPtr& msg);

    //ROS Publish Functions (robot_data_[X] to ros topics)
    void publishFastRateData();
    void publishMedRateData();
    void publishSlowRateData();
    void publishOdometry(float left_vel, float right_vel);
    void publishMotorSpeeds();
    void publishWheelVels();

    //Serial Com Functions
    int getParameterData(int parameter);
    bool setParameterData(int param1, int param2);
    void updateRobotData(int parameter);
    bool sendCommand(int param1, int param2);
    int readCommand();
};

rr_openrover_basic::SmartBatteryStatus interpret_battery_status(uint16_t bits);

}
#endif /* _openrover_hpp */
