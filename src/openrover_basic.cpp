#include <fcntl.h>
#include <termios.h>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
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
    ROS_INFO("Creating Publishers and Subscribers");
    //encoder_pub = nh.advertise<nav_msgs::Odometry>("odom", 2);
    //battery_soc_pub = nh.advertise<std_msgs::Int8("battery_soc",2);
    cmd_vel_sub = nh.subscribe("cmd_vel", 1, &OpenRover::cmdVelCB, this);
    return true;
}

void OpenRover::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("cmd_vel said: %f, %f", msg->linear.x, msg->angular.z);
    int left_motor_speed, right_motor_speed;
    left_motor_speed = (int)((msg->linear.x*30) + (msg->angular.z*20) + 125)%250;
    right_motor_speed =(int)((msg->linear.x*30) - (msg->angular.z*20) + 125)%250;
    ROS_INFO("Converted motor speeds to: %i, %i", left_motor_speed, right_motor_speed);
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

    fd_options.c_cflag |= ( CREAD | CLOCAL | CS8 );
    fd_options.c_cflag &= ~( PARODD | CRTSCTS | CSTOPB | PARENB );
    fd_options.c_iflag &= ~( IUCLC | IXANY | IMAXBEL | IXON | IXOFF | IUTF8 | ICRNL | INPCK );
    fd_options.c_oflag |= ( NL0 | CR0 | TAB0 | BS0 | VT0 | FF0 );
    fd_options.c_oflag &= ~( OPOST | ONLCR | OLCUC | OCRNL | ONOCR | ONLRET | OFILL | OFDEL | NL1 | CR1 | CR2 | TAB3 | BS1 | VT1 | FF1 );
    fd_options.c_lflag |= ( NOFLSH );
    fd_options.c_lflag &= ~( ICANON | IEXTEN | TOSTOP | ISIG | ECHOPRT | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE );
    fd_options.c_cc[VINTR] = 0x03;
    fd_options.c_cc[VQUIT] = 0x1C;
    fd_options.c_cc[VERASE] = 0x7F;
    fd_options.c_cc[VKILL] = 0x15;
    fd_options.c_cc[VEOF] = 0x04;
    fd_options.c_cc[VTIME] = 0x01;
    fd_options.c_cc[VMIN] = 0x00;
    fd_options.c_cc[VSWTC] = 0x00;
    fd_options.c_cc[VSTART] = 0x11;
    fd_options.c_cc[VSTOP] = 0x13;
    fd_options.c_cc[VSUSP] = 0x1A;
    fd_options.c_cc[VEOL] = 0x00;
    fd_options.c_cc[VREPRINT] = 0x12;
    fd_options.c_cc[VDISCARD] = 0x0F;
    fd_options.c_cc[VWERASE] = 0x17;
    fd_options.c_cc[VLNEXT] = 0x16;
    fd_options.c_cc[VEOL2] =  0x00;

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
    unsigned char write_buffer[7], read_buffer[5], test;
    int checksum;    

    write_buffer[0] = 0xfd;
    write_buffer[1] = (char)left_motor_speed;
    write_buffer[2] = (char)right_motor_speed;
    write_buffer[3] = (char)flipper_motor_speed;
    write_buffer[4] = 0x0a; //read data
    write_buffer[5] = 0x28; //encoder2

    //write_buffer[1] = 0x00
    //write_buffer[2] = 0x00
    //write_buffer[3] = 0x00
    //write_buffer[4] = 0x0a; //read data
    //write_buffer[5] = 0x1C; //encoder1


    //Calculate Checksum
    checksum = 255-(write_buffer[1]+write_buffer[2]+write_buffer[3]+write_buffer[4]+write_buffer[5])%255;
    write_buffer[6] = (char)checksum;
    ROS_INFO("I sent: %02x,%02x,%02x,%02x,%02x,%02x,%02x", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);
    ROS_INFO("I sent: %i,%i,%i,%i,%i,%i,%i", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);

    speed_t cfgetispeed(const struct termios *attribs);        
    write(fd, write_buffer, 7);
    read(fd, read_buffer, 5);

    ROS_WARN("I Heard: %02x,%02x,%02x,%02x,%02x", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
    ROS_WARN("I Heard: %i,%i,%i,%i,%i", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
}

int main( int argc, char *argv[] )
{
        // Create ROS node handlers 
        ros::NodeHandle *nh = NULL;
        ros::NodeHandle *nh_priv = NULL;

        // Create driver object
        OpenRover *openrover = NULL;
 
        // Create ROS node
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


        ros::Rate loop_rate(10);

        while(ros::ok())
        {
            //encoder_pub.publish(openrover->readEncoders());
            //battery_soc_pub.publish(openrover->readBattrySOC());
            ros::spinOnce();
            loop_rate.sleep();
        }

        delete openrover;
        delete nh_priv;
        delete nh;

        return 0;
}

