<<<<<<< HEAD

#include <fcntl.h>
#include <termios.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


class OpenRover
{
public:
    OpenRover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
    bool openComms();
    bool spinMotors();
private:
    std::string port;
    int baud;
    int fd;
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;
};

OpenRover::OpenRover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
    port("/dev/ttyUSB0"),
    baud(57600)
{
    ROS_INFO( "Initializing" );
    nh_priv.param( "port", port, (std::string)"/dev/ttyUSB0" );
    nh_priv.param( "baud", baud, 57600 );

    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &OpenRover::cmdVelCB, this);
}

void OpenRover::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("I heard: %f, %f", msg->linear.x, msg->angular.z);
}

bool OpenRover::openComms()
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

bool OpenRover::spinMotors()
{
    unsigned char buffer[6];
    if (openComms())
    {
        buffer[0] = 0xfd;
        buffer[1] = 0x64;
        buffer[2] = 0x64;
        buffer[3] = 0x00;
        buffer[4] = 0x00;
        buffer[5] = 0x00;
        buffer[6] = 0x37;
        ROS_INFO("Attempting to spin motors");
        speed_t cfgetispeed(const struct termios *attribs);
//        ROS_INFO("Baud rate set to: %i", speed_t);
        while(true)
            write(fd, buffer, 7); 
        for (int i=0; i<7; i++)
        { 
            ROS_INFO("I sent %i", buffer[i]);
        }        
        ROS_INFO("Did the motors spin yet?");
        //if (0 > write(fd, buffer, 7))
	//{
	//   ROS_ERROR("Failed to update channel: %s", strerror(errno));
	//    return false;
	//}    
    }
}

int main(int argc, char **argv)
{
    ros::NodeHandle *nh = NULL;
    ros::NodeHandle *nh_priv = NULL;
    OpenRover *openrover = NULL;
    
    ROS_INFO("Creating node");
    ros::init(argc, argv, "openrover_node");
    
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
    openrover->spinMotors();    
    ROS_INFO("ROS spin");
    ros::spin();

    delete openrover;
    delete nh;
    delete nh_priv;

    return 0;
}
=======
#include "kangaroo_driver/kangaroo_driver.hpp"
#include "kangaroo_driver/kang_lib.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <cmath>

//namespace kangaroo
//{

openrover::openrover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
	port( "" ),
	fd( -1 ),
	nh( _nh ),
	nh_priv( _nh_priv ),
	encoder_lines_per_revolution(3200),
	hz(50)
	//diameter_of_wheels(.117475)
	//msg(new sensor_msgs::JointState)
{
	ROS_INFO( "Initializing Rover" );
	nh_priv.param( "port", port, (std::string)"/dev/serial" );
    nh_priv.param( "baud_rate", baud_rate, 57600 );

	// the rate we want to set the timer at
	double rate = (double)1/(double)hz;

	poll_timer = nh.createWallTimer( ros::WallDuration(rate), &kangaroo::JointStateCB, this );

	//circumference_of_wheels = diameter_of_wheels * M_PI;
}

bool openrover::open()
{
	struct termios fd_options;

	if( is_open() )
	{
		ROS_INFO( "Port is already open - Closing to re-open" );
		close();
	}

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
	if( 0 > cfsetispeed( &fd_options, B38400) )
	{
		ROS_FATAL( "Failed to set input baud: %s", strerror( errno ) );
		return false;
	}
	if( 0 > cfsetospeed( &fd_options, B38400 ) )
	{
		ROS_FATAL( "Failed to set output baud: %s", strerror( errno ) );
		return false;
	}


	if( 0 > tcsetattr( fd, TCSANOW, &fd_options ) )
	{
		ROS_FATAL( "Failed to set port attributes: %s", strerror( errno ) );
		return false;
	}

	return true;
}

void openrover::close( )
{
	ROS_INFO( "Closing Port" );
	::close( fd );
}

bool openrover::start( )
{
	if( !is_open() && !open() )
		return false;
	ROS_INFO("OpenRover: Starting");
	if( !joint_traj_sub )
		joint_traj_sub = nh.subscribe( "cmd_vel", 1, &openrover::CmdVelCB, this );
	ROS_INFO("OpenRover: Started");
	return true;
}

void openrover::stop( )
{
	ROS_INFO( "OpenRover: Stopping" );

	if( joint_traj_sub )
		joint_traj_sub.shutdown( );

	close( );
}

bool openrover::is_open( ) const
{
	return ( fd >= 0 );
}

void openrover::CmdVelCB(const trajectory_msgs::JointTrajectoryPtr &msg)
{
	int ch1_idx = -1;
	int ch2_idx = -1;

	left_motor_speed = msg->joint_names[i];
	right_motor_spped = msg->
    if (msg->joint_names[i] == ch2_joint_name)
			ch2_idx = i;


	tcflush(fd, TCOFLUSH);

	double channel_1_speed = msg->points[0].velocities[ch1_idx];
	double channel_2_speed = msg->points[0].velocities[ch2_idx];

	channel_1_speed = radians_to_encoder_lines(channel_1_speed);
	channel_2_speed = radians_to_encoder_lines(channel_2_speed);

	// lock the output_mutex
	boost::mutex::scoped_lock output_lock(output_mutex);
	set_motor_speed(left, 128, '1');
	set_motor_speed(right, 128, '2');
    set_motor_speed(flipper, 128, '2');
}



>>>>>>> 5cf2938d8aec2b85a5b60b21e852796136dcb5ca
