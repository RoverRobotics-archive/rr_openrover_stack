#include <fcntl.h>
#include <termios.h>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>
#include "nav_msgs/Odometry.h"
#include "rr_openrover_basic/openrover.hpp"

const int START_BYTE = 253;
const int i_OUT_PACKAGE_LENGTH = 7;

const int i_REG_PWR_TOTAL_CURRENT_INDEX = 0;
const int i_REG_MOTOR_FB_RPM_LEFT_INDEX = 2;
const int i_REG_MOTOR_FB_RPM_RIGHT_INDEX = 4;
const int i_REG_FLIPPER_FB_POSITION_POT1 = 6;
const int i_REG_FLIPPER_FB_POSITION_POT2 = 8;
const int i_REG_MOTOR_FB_CURRENT_LEFT = 10;
const int i_REG_MOTOR_FB_CURRENT_RIGHT = 12;
const int i_REG_MOTOR_FAULT_FLAG_LEFT = 18;
const int i_REG_MOTOR_TEMP_LEFT = 20;
const int i_REG_MOTOR_TEMP_RIGHT = 22;
const int i_REG_POWER_BAT_VOLTAGE_A = 24;
const int i_REG_POWER_BAT_VOLTAGE_B = 26;
const int i_ENCODER_INTERVAL_MOTOR_LEFT = 28;
const int i_ENCODER_INTERVAL_MOTER_RIGHT = 20;
const int i_ENCODER_INTERVAL_MOTOR_FLIPPER = 32;
const int i_REG_ROBOT_REL_SOC_A = 34;
const int i_REG_ROBOT_REL_SOC_B = 36;
const int i_REG_MOTOR_CHARGER_STATE = 38;
const int i_BUILDNO = 40;
const int i_REG_POWER_A_CURRENT = 42;
const int i_REG_POWER_B_CURRENT = 44;
const int i_REG_MOTOR_FLIPPER_ANGLE = 46;
const int i_to_computer_REG_MOTOR_SIDE_FAN_SPEED = 48;
const int i_to_computer_REG_MOTOR_SLOW_SPEED = 50;

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
    x_button_sub = nh.subscribe("joystick/x_button", 1, &OpenRover::toggleLowSpeedMode, this);
    return true;
}

void OpenRover::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("cmd_vel said: %f, %f", msg->linear.x, msg->angular.z);
    int left_motor_speed, right_motor_speed, flipper_motor_speed;
    left_motor_speed = (int)((msg->linear.x*30) + (msg->angular.z*20) + 125)%250;
    right_motor_speed =(int)((msg->linear.x*30) - (msg->angular.z*20) + 125)%250;
    flipper_motor_speed = (int)((msg->angular.y*20) + 125)%250;
    //ROS_INFO("Converted motor speeds to: %i, %i", left_motor_speed, right_motor_speed);
   
   	output_mutex.lock();
	input_mutex.lock();
    setMotorSpeed(left_motor_speed, right_motor_speed, flipper_motor_speed);
    output_mutex.unlock();
	input_mutex.unlock();
}

bool OpenRover::setMotorSpeed(int left_motor_speed, int right_motor_speed, int flipper_motor_speed)
{
	updateMotorSpeeds(left_motor_speed, right_motor_speed, flipper_motor_speed);
    unsigned char write_buffer[7], read_buffer[5], test;
    int checksum;
    static int timer1 = 0;
    static int j = 0;
    int val_tot;
    //static std::time_t start = 0;
    double duration;
    
    static struct timespec ts1;
    static struct timespec ts2;
    
    timer1 = (timer1 + 1)%40; //every 4s reset
    write_buffer[0] = 0xfd;
    write_buffer[1] = (char)left_motor_speed;
    write_buffer[2] = (char)right_motor_speed;
    write_buffer[3] = (char)flipper_motor_speed;
    write_buffer[4] = 10; //Param 1: 10 to get data, 240 to set low speed mode 
    write_buffer[5] = 28; //Param 2:  -> 28=left mot, 30=right mot

	if(timer1==0) // every 
	{
		//write_buffer[4] = 240; //Param 1: 10 to get data, 240 for low speed mode 
		j = (j+1)%2;
		//write_buffer[5] = j;
	} 	

    //Calculate Checksum
    checksum = 255-(write_buffer[1]+write_buffer[2]+write_buffer[3]+write_buffer[4]+write_buffer[5])%255;
    write_buffer[6] = (char)checksum;
    
    //ROS_INFO("I sent: %02x,%02x,%02x,%02x,%02x,%02x,%02x", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);
    //ROS_INFO("I sent: %i,%i,%i,%i,%i,%i,%i", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);
    //ROS_INFO("Param2: %i, S_Byte: %i", write_buffer[5], write_buffer[0]);
    //speed_t cfgetispeed(const struct termios *attribs);
    //high_resolution_clock::time_point t1 = high_resolution_clock::now();
    //std::time_t t1 = std::time(NULL);
    timespec_get(&ts1, TIME_UTC);
    write(fd, write_buffer, 7);
    read(fd, read_buffer, 5);
    //write(fd, write_buffer, 7);
    //read(fd, read_buffer, 5);
    timespec_get(&ts2, TIME_UTC);
    double t2 = (double)ts2.tv_sec + (double)ts2.tv_nsec/1000000000.0;
    double t1 = (double)ts1.tv_sec + (double)ts1.tv_nsec/1000000000.0;
    duration = t2-t1;
    //std::time_t t2 = std::time(NULL);
    //high_resolution_clock::time_point t2 = high_resolution_clock::now();
    //duration = duration_cast<duration<double>>(t2-t1);
    //double int_t2 = (double)t2;
    //double int_t1 = (double)t1;
    //duration = int_t2-int_t1;
 
    val_tot = (read_buffer[2]<<8) + read_buffer[3];
    //ROS_WARN("I Heard: %02x,%02x,%02x,%02x,%02x", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
    //ROS_WARN("%f Param: %i, Val: %i, Timer: %i, S_Byte: %i", (t2-t1), read_buffer[1], read_buffer[3], timer1, read_buffer[0]);
    int crc_calc = 255-(read_buffer[1]+read_buffer[2]+read_buffer[3])%255;
    int crc_good = crc_calc-read_buffer[4];
    ROS_WARN("CRC_good: %i, Val: %i, S_Byte: %i", crc_good, val_tot, read_buffer[0]);
}

void OpenRover::toggleLowSpeedMode(const std_msgs::Bool::ConstPtr& msg)
{
	int mode_setting;
	
	mode_setting = getParameterData(28);
	
	/*if(mode_setting==1)
	{
		ROS_INFO("Switching off low speed mode.");
		if(!setParameterData(240, 0))
		{
			ROS_ERROR("Failed to turn off low speed mode.");
		}
	} else if(mode_setting==0) {
		ROS_INFO("Switching on low speed mode.");
		if(!setParameterData(240, 1))
		{
			ROS_ERROR("Failed to turn on low speed mode.");
		}
	} else {
		ROS_ERROR("toggleLowSpeedMode error. Mode setting not valid. Defaulting to off.");
		if(!setParameterData(240, 0))
		{
			ROS_ERROR("Failed to turn off low speed mode.");
		}
	}*/
}

bool OpenRover::sendCommand(int param1, int param2)
{
	unsigned int write_buffer[7];
	unsigned int value;
	
    write_buffer[0] = START_BYTE;
    write_buffer[1] = (char)motor_speeds[0]; //left motor
    write_buffer[2] = (char)motor_speeds[1]; //right motor
    write_buffer[3] = (char)motor_speeds[2]; //flipper
    write_buffer[4] = (char)param1; //Param 1: 10 to get data, 240 for low speed mode 
    write_buffer[5] = (char)param2; //Param 2:  -> 28=left mot, 30=right mot
    //Calculate Checksum
    write_buffer[6] = (char) 255-(write_buffer[1]+write_buffer[2]+write_buffer[3]+write_buffer[4]+write_buffer[5])%255;
	
	if (0>write(fd, write_buffer, 7))
	{
		ROS_ERROR("Failed to send command %02x,%02x,%02x,%02x,%02x,%02x,%02x", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);
		return false;
	}
	return true;
}

int OpenRover::readCommand() //must be used after a send command
{
	int read_buffer[5];
	int value, checksum;
	int bits_read = read(fd, read_buffer, 5);
	if(bits_read <5)
	{
		bits_read = read(fd, read_buffer, 5);
		if (1>bits_read)
		{
			ROS_ERROR("Reading from serial critically failed: %s", strerror(errno));
			return -1;
		}
	}
	ROS_INFO("bits %i", bits_read);
	
	if(!(START_BYTE==read_buffer[0]))
	{		
		ROS_ERROR("Incorrect start byte: %i", read_buffer[4]);
		return -1;		
	}
	checksum = 255-(read_buffer[1]+read_buffer[2]+read_buffer[3])%255;
	if(!(checksum==read_buffer[4]))
	{		
		ROS_ERROR("Received bad CRC, check: %i, rec: %i", checksum, read_buffer[4]);
		return -1;
	}	
	
	return value;
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

void OpenRover::updateAllRobotData(){
	//int motor_speeds[3] = {left_motor_speed, right_motor_speed, flipper_motor_speed};
	//robot_data
/*const int  = 2;
const int i_REG_MOTOR_FB_RPM_RIGHT_INDEX = 4;
const int i_REG_FLIPPER_FB_POSITION_POT1 = 6;
const int i_REG_FLIPPER_FB_POSITION_POT2 = 8;
const int i_REG_MOTOR_FB_CURRENT_LEFT = 10;
const int i_REG_MOTOR_FB_CURRENT_RIGHT = 12;
const int i_REG_MOTOR_FAULT_FLAG_LEFT = 18;
const int i_REG_MOTOR_TEMP_LEFT = 20;
const int i_REG_MOTOR_TEMP_RIGHT = 22;
const int i_REG_POWER_BAT_VOLTAGE_A = 24;
const int i_REG_POWER_BAT_VOLTAGE_B = 26;
const int i_ENCODER_INTERVAL_MOTOR_LEFT = 28;
const int i_ENCODER_INTERVAL_MOTER_RIGHT = 20;
const int i_ENCODER_INTERVAL_MOTOR_FLIPPER = 32;
const int i_REG_ROBOT_REL_SOC_A = 34;
const int i_REG_ROBOT_REL_SOC_B = 36;
const int i_REG_MOTOR_CHARGER_STATE = 38;
const int i_BUILDNO = 40;
const int i_REG_POWER_A_CURRENT = 42;
const int i_REG_POWER_B_CURRENT = 44;
const int i_REG_MOTOR_FLIPPER_ANGLE = 46;
 */
	updateRobotData(i_REG_PWR_TOTAL_CURRENT_INDEX);
	updateRobotData(i_REG_FLIPPER_FB_POSITION_POT1);
	updateRobotData(i_REG_FLIPPER_FB_POSITION_POT2);
	updateRobotData(i_REG_MOTOR_FB_CURRENT_LEFT);
	updateRobotData(i_REG_MOTOR_FB_CURRENT_RIGHT);	
}

void OpenRover::updateMotorSpeeds(int left_motor, int right_motor, int flipper_motor)
{
	motor_speeds[0] = left_motor;
	motor_speeds[1] = right_motor;
	motor_speeds[2] = flipper_motor;
}

void OpenRover::updateRobotData(int param)
{
	robot_data[param] = getParameterData(param);
}

bool OpenRover::setParameterData(int param1, int param2)
{
	output_mutex.lock();
	input_mutex.lock();
	
	if(!sendCommand(param1, param2))
	{		
		ROS_ERROR("Failed sendCommand while setting parameter %i to %i", param1, param2);
		return false;
	}
	output_mutex.unlock();
	input_mutex.unlock();
	
	if(!(param2 == getParameterData(param1)))
	{
		ROS_ERROR("Failed to set parameter %i to %i", param1, param2);
		return false;
	}
	return true;
}

int OpenRover::getParameterData(int param){
	//unsigned int read_buffer[5];
	int value;
	
	output_mutex.lock();
	input_mutex.lock();
    /*
    for (int i=0; i < 3; i++)
    {
		sendCommand(10, param);
		read(fd, read_buffer, 5);
		checksum_receive = 255-(read_buffer[1]+read_buffer[2]+read_buffer[3])%255;
		if(!(checksum_receive==read_buffer[4]))
		{		
			ROS_WARN("Bad CRC %i - Reattempting", i);	
		}
		if(i==2)
		{
			ROS_ERROR("Failed to get parameter %i", param);
			return 2147483647;
		}
	}
	*/
	if(!sendCommand(10, param))
	{
		ROS_ERROR("Failed sendCommand while getting parameter %i", param);
		output_mutex.unlock();
		input_mutex.unlock();
		return -1;
	}
	
	output_mutex.unlock();
	value = readCommand();
	
	if(0>value)
	{
		ROS_ERROR("Failed readCommand while getting parameter %i", param);
		input_mutex.unlock();
		return -1;
	}
	input_mutex.unlock();
	
    return value;
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

