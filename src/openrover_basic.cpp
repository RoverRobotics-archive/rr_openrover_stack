#include <fcntl.h>
#include <termios.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rr_openrover_basic/openrover.hpp"

OpenRover::OpenRover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
    port("/dev/ttyUSB0"),
    baud(57600)
{
    ROS_INFO( "Initializing" );
    nh_priv.param( "port", port, (std::string)"/dev/ttyUSB0" );
    nh_priv.param( "baud", baud, 57600 );
}

bool OpenRover::start()
{
    openComs();
    ROS_INFO("Creating Subscriber");
    cmd_vel_sub = nh.subscribe("cmd_vel", 1, &OpenRover::cmdVelCB, this);
    return true;
}

void OpenRover::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("I heard: %f, %f", msg->linear.x, msg->angular.z);
    int left_motor_speed, right_motor_speed;
    left_motor_speed = (msg->linear.x*30) + (msg->angular.z*20) + 125;
    right_motor_speed = (msg->linear.x*30) - (msg->angular.z*20) + 125;
    setMotorSpeed(left_motor_speed, right_motor_speed, 125);    
}

bool OpenRover::openComs()
{
    ROS_INFO("Opening serial port");
    struct termios fd_options;
    fd = ::open( port.c_str( ), O_RDWR | O_NOCTTY | O_NDELAY );
    if( fd < 0 )
    {
	ROS_FATAL( "Failed to open port: %s", strerror( errno ) );
	return false;
    }
    if( 0 > fcntl( fd, F_SETFL, 0 ) )
    {
	ROS_FATAL( "Failed to set port descriptor: %s", strerror( errno ) );
  	return false;
    }
    if( 0 > tcgetattr( fd, &fd_options ) )
    {
	ROS_FATAL( "Failed to fetch port attributes: %s", strerror( errno ) );
	return false;
    }
    if( 0 > cfsetispeed( &fd_options, B57600 ) )
    {
	ROS_FATAL( "Failed to set input baud: %s", strerror( errno ) );
        return false;
    }
    if( 0 > cfsetospeed( &fd_options, B57600 ) )
    {
        ROS_FATAL( "Failed to set output baud: %s", strerror( errno ) );
        return false;
    }

    if( 0 > tcsetattr( fd, TCSANOW, &fd_options ) )
    {
	ROS_FATAL( "Failed to set port attributes: %s", strerror( errno ) );
	return false;
    }
    ROS_INFO("Serial port opened");
    return true;
}

bool OpenRover::setMotorSpeed(int left_motor_speed, int right_motor_speed, int flipper_motor_speed)
{
    unsigned char buffer[6];
    if (true)
    {
        buffer[0] = 0xfd;
        buffer[1] = (char)left_motor_speed;
        buffer[2] = (char)right_motor_speed;
        buffer[3] = (char)flipper_motor_speed;
        buffer[4] = 0x00;
        buffer[5] = 0x00;

        //Calculate Checksum
        int checksum;
        checksum = 255-(buffer[1]+buffer[2]+buffer[3])%255;
        ROS_INFO("Integer checksum: %i", checksum);
        buffer[6] = (char)checksum;
        //ROS_INFO("Byte checksum: %p", buffer[6]); 
        ROS_INFO("I sent: %p", buffer);

        speed_t cfgetispeed(const struct termios *attribs);        
        write(fd, buffer, 7); 
    }
}

int main( int argc, char *argv[] )
{
        ros::NodeHandle *nh = NULL;
        ros::NodeHandle *nh_priv = NULL;
        OpenRover *openrover = NULL;

        ROS_INFO("Creating node");
        ros::init( argc, argv, "openrover_basic_node" );

        nh = new ros::NodeHandle( );
        if( !nh )
        {
                ROS_FATAL( "Failed to initialize NodeHandle" );
                ros::shutdown( );
                return -1;
        }
        nh_priv = new ros::NodeHandle( "~" );
        if( !nh_priv )
        {
                ROS_FATAL( "Failed to initialize private NodeHandle" );
                delete nh;
                ros::shutdown( );
                return -2;
        }
        openrover = new OpenRover( *nh, *nh_priv );
        if( !openrover )
        {
                ROS_FATAL( "Failed to initialize driver" );
                delete nh_priv;
                delete nh;
                ros::shutdown( );
                return -3;
        }
        if( !openrover->start( ) )
                ROS_ERROR( "Failed to start the driver" );

        ros::spin( );

        delete openrover;
        delete nh_priv;
        delete nh;

        return 0;
}

