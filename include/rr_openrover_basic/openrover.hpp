#ifndef _openrover_hpp
#define _openrover_hpp

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <stdint.h>
#include <boost/thread.hpp>
#include <string>
#include <fcntl.h>
#include <termios.h>


class OpenRover
{
public:
    OpenRover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );
    bool start();
    bool openComs();
    bool commandMotors(int left_motor_speed, int right_motor_speed, int flipper_motor_speed);
    bool setMotorSpeed(int left_motor_speed, int right_motor_speed, int flipper_motor_speed);

private:
    //ROS Parameters
    std::string port;

    //ROS node handlers
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    //ROS Publisher and Subscribers
    ros::Publisher encoder_pub;
    ros::Publisher battery_soc_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber x_button_sub;

    //General Class variables
    int baud;
    int fd;	
	int robot_data[50];
	int motor_speeds[3];
    //ROS Subscriber callback functions
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
    void toggleLowSpeedMode(const std_msgs::Bool::ConstPtr& msg);
    void updateAllRobotData();
    int getParameterData(int parameter);
    bool setParameterData(int param1, int param2);
    void updateRobotData(int parameter);
    void updateMotorSpeeds(int left_motor_speed, int right_motor_speed, int flipper_motor_speed);
    bool sendCommand(int param1, int param2);
    int readCommand();
    // mutex-es for accessing the serial port
    // output_muxes must be locked first
    boost::mutex output_mutex;
    boost::mutex input_mutex;
};


#endif /* _openrover_hpp */
