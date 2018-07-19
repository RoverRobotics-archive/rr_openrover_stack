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
#include <rr_openrover_basic/RawRrOpenroverBasicFastRateData.h>
#include <rr_openrover_basic/RawRrOpenroverBasicMedRateData.h>
#include <rr_openrover_basic/RawRrOpenroverBasicSlowRateData.h>

class OpenRover
{
public:
    OpenRover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );
    bool start();
    bool openComs();
    
    void serialManager();
    
    void robotDataFastCB( const ros::WallTimerEvent &e);
    void robotDataMediumCB( const ros::WallTimerEvent &e);
    void robotDataSlowCB( const ros::WallTimerEvent &e);
    void timeoutCB( const ros::WallTimerEvent &e );
    
    bool publish_fast_rate_vals_;
    bool publish_med_rate_vals_;
    bool publish_slow_rate_vals_;
    bool low_speed_mode_on_;

private:
    //ROS Parameters
    std::string port_, drive_type_;
    
    float timeout_; //Default to neutral motor values after timeout seconds

    //ROS node handlers
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;
    
    //ROS Timers
    ros::WallTimer fast_timer;
    ros::WallTimer medium_timer;
    ros::WallTimer slow_timer;
    ros::WallTimer timeout_timer;

    //ROS Publisher and Subscribers
    ros::Publisher odom_enc_pub;
    ros::Publisher battery_state_pub;
    
    ros::Publisher fast_rate_pub;
    ros::Publisher medium_rate_pub;
    ros::Publisher slow_rate_pub;
    
    ros::Subscriber cmd_vel_sub;

    //General Class variables
    int baud_; //serial baud rate
    int fd;
    int robot_data_[50]; //stores all received data from robot
    char motor_speeds_commanded_[3]; //stores most recent commanded motor speeds
    double fast_rate_; //update rate for encoders, 10Hz recommended
    double medium_rate_;
    double slow_rate_;
    std::vector<char> serial_fast_buffer_;
    std::vector<char> serial_medium_buffer_;
    std::vector<char> serial_slow_buffer_;

    //ROS Subscriber callback functions
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
    
    //ROS Publish Functions (robot_data_[X] to ros topics)
    void publishFastRateData();
    void publishMedRateData();
    void publishSlowRateData();
    void publishOdomEnc();
    
    //Serial Com Functions
    int getParameterData(int parameter);
    bool setParameterData(int param1, int param2);
    void updateRobotData(int parameter);
    void updateMotorSpeedsCommanded(char left_motor_speed, char right_motor_speed, char flipper_motor_speed);
    bool sendCommand(int param1, int param2);
    int readCommand();
    
    //Odometry Mapping Functions- WIP
    void parseWheelType();
};


#endif /* _openrover_hpp */
