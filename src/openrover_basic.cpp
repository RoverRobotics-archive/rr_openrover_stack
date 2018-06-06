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



