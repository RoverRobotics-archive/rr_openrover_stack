#ifndef _openrover_hpp
#define _openrover_hpp

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
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

    //General Class variables
    int baud;
    int fd;

    //ROS Subscriber callback functions
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
};


#endif /* _openrover_hpp */
