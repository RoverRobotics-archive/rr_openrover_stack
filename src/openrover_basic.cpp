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
