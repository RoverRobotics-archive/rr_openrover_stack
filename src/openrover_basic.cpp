#include <fcntl.h>
#include <termios.h>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>
#include "nav_msgs/Odometry.h"
#include "rr_openrover_basic/openrover.hpp"
#include <boost/thread.hpp>
#include <vector>

#include <rr_openrover_basic/RawRrOpenroverBasicEncoders.h>
#include <rr_openrover_basic/RawRrOpenroverBasicMedRateData.h>
#include <rr_openrover_basic/RawRrOpenroverBasicSlowRateData.h>

const int LOOP_RATE = 1000;
const int START_BYTE = 253;
const int i_OUT_PACKAGE_LENGTH = 7;

//Firmware parameters. Maintained numbering to keep consistency.
const int i_REG_PWR_TOTAL_CURRENT_INDEX = 0;  //5hz
const int i_REG_MOTOR_FB_RPM_LEFT_INDEX = 2; //5hz
const int i_REG_MOTOR_FB_RPM_RIGHT_INDEX = 4; //5hz
const int i_REG_FLIPPER_FB_POSITION_POT1 = 6; //5hz
const int i_REG_FLIPPER_FB_POSITION_POT2 = 8; //5hz 

const int i_REG_MOTOR_FB_CURRENT_LEFT = 10; //5hz
const int i_REG_MOTOR_FB_CURRENT_RIGHT = 12; //5hz
const int i_REG_MOTOR_FAULT_FLAG_LEFT = 18; //1hz
const int i_REG_MOTOR_TEMP_LEFT = 20; //1hz
const int i_REG_MOTOR_TEMP_RIGHT = 22; //1hz

const int i_REG_POWER_BAT_VOLTAGE_A = 24; //1hz
const int i_REG_POWER_BAT_VOLTAGE_B = 26; //1hz
const int i_ENCODER_INTERVAL_MOTOR_LEFT = 28; //10hz
const int i_ENCODER_INTERVAL_MOTER_RIGHT = 30; //10hz
const int i_ENCODER_INTERVAL_MOTOR_FLIPPER = 32; //10hz

const int i_REG_ROBOT_REL_SOC_A = 34; //1hz
const int i_REG_ROBOT_REL_SOC_B = 36; //1hz
const int i_REG_MOTOR_CHARGER_STATE = 38;  //5hz
const int i_BUILDNO = 40;  //1hz
const int i_REG_POWER_A_CURRENT = 42;  //5hz

const int i_REG_POWER_B_CURRENT = 44; //5hz
const int i_REG_MOTOR_FLIPPER_ANGLE = 46;  //5hz
//const int i_to_computer_REG_MOTOR_SIDE_FAN_SPEED = 48; //5hz
//const int i_to_computer_REG_MOTOR_SLOW_SPEED = 50; //5hz

const int ROBOT_DATA_INDEX_FAST[] = {
i_ENCODER_INTERVAL_MOTOR_LEFT, i_ENCODER_INTERVAL_MOTER_RIGHT}; 
//flipped encoder not in use now, i_ENCODER_INTERVAL_MOTOR_FLIPPER}; //10hz

const int ROBOT_DATA_INDEX_MEDIUM[] = {
i_REG_PWR_TOTAL_CURRENT_INDEX, i_REG_MOTOR_FB_RPM_LEFT_INDEX,
i_REG_MOTOR_FB_RPM_RIGHT_INDEX, i_REG_FLIPPER_FB_POSITION_POT1, i_REG_FLIPPER_FB_POSITION_POT2,
i_REG_MOTOR_FB_CURRENT_LEFT, i_REG_MOTOR_FB_CURRENT_RIGHT, i_REG_MOTOR_CHARGER_STATE,
i_REG_POWER_A_CURRENT, i_REG_POWER_B_CURRENT, i_REG_MOTOR_FLIPPER_ANGLE
//i_to_computer_REG_MOTOR_SIDE_FAN_SPEED, i_to_computer_REG_MOTOR_SLOW_SPEED
};

const int ROBOT_DATA_INDEX_SLOW[] = {
i_REG_MOTOR_FAULT_FLAG_LEFT,i_REG_MOTOR_TEMP_LEFT,
i_REG_MOTOR_TEMP_RIGHT, i_REG_POWER_BAT_VOLTAGE_A, i_REG_POWER_BAT_VOLTAGE_B,
i_REG_ROBOT_REL_SOC_A, i_REG_ROBOT_REL_SOC_B, i_BUILDNO};

const int FAST_SIZE = sizeof(ROBOT_DATA_INDEX_FAST)/sizeof(ROBOT_DATA_INDEX_FAST[0]);
const int MEDIUM_SIZE = sizeof(ROBOT_DATA_INDEX_MEDIUM)/sizeof(ROBOT_DATA_INDEX_MEDIUM[0]);
const int SLOW_SIZE = sizeof(ROBOT_DATA_INDEX_SLOW)/sizeof(ROBOT_DATA_INDEX_SLOW[0]);

OpenRover::OpenRover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
    port("/dev/ttyUSB0"),
    baud(57600),
    wheel_type(1),
    poll_rate(1.0/75.0),
    fast_rate(1.0/10.0),
    medium_rate(1.0/2.0),
    slow_rate(1.0/1.0),
    motor_speeds{125,125,125},
    publish_encoder_vals(false),
    publish_med_rate_vals(false),
    publish_slow_rate_vals(false)
{
    ROS_INFO( "Initializing" );
    nh_priv.param( "port", port, (std::string)"/dev/ttyUSB0" );
    nh_priv.param( "baud", baud, 57600 );
    
    serial_vect_buffer.reserve(50); //reserve space for 25 param pairs
    serial_fast_buffer.reserve(10*FAST_SIZE); //reserve space for 2 sets of 3 readings
    serial_medium_buffer.reserve(10*MEDIUM_SIZE); //reserve space for 2 sets of 3 readings
    serial_slow_buffer.reserve(10*SLOW_SIZE); //reserve space for 2 sets of 3 readings
    
    encoder_timer = nh.createWallTimer( ros::WallDuration(fast_rate), &OpenRover::EncoderTimerCB, this);
    medium_timer = nh.createWallTimer( ros::WallDuration(medium_rate), &OpenRover::RobotDataMediumCB, this);
    slow_timer = nh.createWallTimer( ros::WallDuration(slow_rate), &OpenRover::RobotDataSlowCB, this);
}

bool OpenRover::start()
{
    openComs();
    ROS_INFO("Creating Publishers and Subscribers");
    //odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 2);
    //encoder_pub = nh.advertise<std_msgs::Int16>("encoders", 1);    
    encoder_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicEncoders>("raw_encoders",2);
    medium_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicMedRateData>("raw_med_rate_data",2);
    slow_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicSlowRateData>("raw_slow_rate_data",2);
    //battery_soc_pub = nh.advertise<std_msgs::Int8("battery_soc",2);
    cmd_vel_sub = nh.subscribe("cmd_vel", 1, &OpenRover::cmdVelCB, this);
    x_button_sub = nh.subscribe("joystick/x_button", 1, &OpenRover::toggleLowSpeedMode, this);
    
    setParameterData(240, 1); //turn low speed on to keep robot from running away
    return true;
}

void OpenRover::RobotDataSlowCB(const ros::WallTimerEvent &e)
{
	//ROS_INFO("Slow called");
	for(int i = 0; i<SLOW_SIZE; i++)
	{
		serial_slow_buffer.push_back(10);
		serial_slow_buffer.push_back(ROBOT_DATA_INDEX_SLOW[i]);
	}
	publish_slow_rate_vals = true;
}

void OpenRover::RobotDataMediumCB(const ros::WallTimerEvent &e)
{
	//ROS_INFO("Medium called");
	for(int i = 0; i<MEDIUM_SIZE; i++)
	{
		serial_medium_buffer.push_back(10);
		serial_medium_buffer.push_back(ROBOT_DATA_INDEX_MEDIUM[i]);
	}
	publish_med_rate_vals = true;
}

void OpenRover::EncoderTimerCB(const ros::WallTimerEvent &e)
{
	//ROS_INFO("Encoder called");
	for(int i = 0; i<FAST_SIZE; i++)
	{
		serial_fast_buffer.push_back(10);
		serial_fast_buffer.push_back(ROBOT_DATA_INDEX_FAST[i]);
	}
	publish_encoder_vals = true;
}

int OpenRover::readEncoders()
{
	//int encoder_vals[3];
	int encoder_left, encoder_right, encoder_flip;
	encoder_left = robot_data[i_ENCODER_INTERVAL_MOTOR_LEFT];
	encoder_right = robot_data[i_ENCODER_INTERVAL_MOTER_RIGHT];
	encoder_flip = robot_data[i_ENCODER_INTERVAL_MOTOR_FLIPPER];
	
	return encoder_left;
}

void OpenRover::publishEncoders()
{
	ROS_INFO("Encoders published");
	int encoder_left, encoder_right, encoder_flip;
	rr_openrover_basic::RawRrOpenroverBasicEncoders msg;
	
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "";
	msg.left_motor = robot_data[i_ENCODER_INTERVAL_MOTOR_LEFT];
	msg.right_motor = robot_data[i_ENCODER_INTERVAL_MOTER_RIGHT];
	msg.flipper_motor = -1; //robot_data[i_ENCODER_INTERVAL_MOTOR_FLIPPER];
	
	//msg_left.data = encoder_left;
	
	encoder_pub.publish(msg);
	publish_encoder_vals = false;
}

void OpenRover::publishMedRateData()
{
	ROS_INFO("Publishing med data rates");
	rr_openrover_basic::RawRrOpenroverBasicMedRateData med_msg;
	
	med_msg.header.stamp = ros::Time::now();
	med_msg.header.frame_id = "";
	med_msg.reg_pwr_total_current = robot_data[i_REG_PWR_TOTAL_CURRENT_INDEX];
	med_msg.reg_motor_fb_rpm_left = robot_data[i_REG_MOTOR_FB_RPM_LEFT_INDEX];
	med_msg.reg_motor_fb_rpm_right = robot_data[i_REG_MOTOR_FB_RPM_RIGHT_INDEX];
	
	med_msg.reg_flipper_fb_position_pot1 = robot_data[i_REG_FLIPPER_FB_POSITION_POT1];
	med_msg.reg_flipper_fb_position_pot2 = robot_data[i_REG_FLIPPER_FB_POSITION_POT2];
	med_msg.reg_motor_fb_current_left = robot_data[i_REG_MOTOR_FB_CURRENT_LEFT];
	
	med_msg.reg_motor_fb_current_right = robot_data[i_REG_MOTOR_FB_CURRENT_RIGHT];
	med_msg.reg_motor_charger_state = robot_data[i_REG_MOTOR_CHARGER_STATE];
	med_msg.reg_power_a_current = robot_data[i_REG_POWER_A_CURRENT];
	
	med_msg.reg_power_b_current = robot_data[i_REG_POWER_B_CURRENT];
	med_msg.reg_motor_flipper_angle = robot_data[i_REG_MOTOR_FLIPPER_ANGLE];
	
	medium_rate_pub.publish(med_msg);
	publish_med_rate_vals = false;
}

void OpenRover::publishSlowRateData()
{
	ROS_INFO("Publishing slow data rate");
	rr_openrover_basic::RawRrOpenroverBasicSlowRateData slow_msg;
	
	slow_msg.header.stamp = ros::Time::now();
	slow_msg.header.frame_id = "";
	slow_msg.reg_motor_fault_flag_left = robot_data[i_REG_MOTOR_FAULT_FLAG_LEFT];
	slow_msg.reg_motor_temp_left = robot_data[i_REG_MOTOR_TEMP_LEFT];
	slow_msg.reg_motor_temp_right = robot_data[i_REG_MOTOR_TEMP_RIGHT];
	
	slow_msg.reg_power_bat_voltage_a = robot_data[i_REG_POWER_BAT_VOLTAGE_A];
	slow_msg.reg_power_bat_voltage_b = robot_data[i_REG_POWER_BAT_VOLTAGE_B];
	slow_msg.reg_robot_rel_soc_a = robot_data[i_REG_ROBOT_REL_SOC_A];	
	
	slow_msg.reg_robot_rel_soc_b = robot_data[i_REG_ROBOT_REL_SOC_B];	
	slow_msg.buildno = robot_data[i_BUILDNO];
	
	slow_rate_pub.publish(slow_msg);
	publish_slow_rate_vals = false;
}

void OpenRover::SerialManager()
{
	char param1;
	char param2;
	
	while ((serial_fast_buffer.size()>1) || (serial_medium_buffer.size()>1) || (serial_slow_buffer.size()>1))
	{
		if (serial_fast_buffer.size()>1)
		{	
			param2 = serial_fast_buffer.back();
			serial_fast_buffer.pop_back();
			
			param1 = serial_fast_buffer.back();
			serial_fast_buffer.pop_back();			
			
		} else if (serial_medium_buffer.size()>1) {
			
			param2 = serial_medium_buffer.back();
			serial_medium_buffer.pop_back();
			
			param1 = serial_medium_buffer.back();
			serial_medium_buffer.pop_back();
			
		} else if (serial_slow_buffer.size()>1) {
			
			param2 = serial_slow_buffer.back();
			serial_slow_buffer.pop_back();
			
			param1 = serial_slow_buffer.back();
			serial_slow_buffer.pop_back();
			
		} else {
			param2 = 0;
			param1 = 0;	
		}
		
		//ROS_INFO("SerialCB called %i, %i, %i, %i", serial_slow_buffer.size(), serial_medium_buffer.size(), serial_fast_buffer.size(), param2);
		//If param1==10, then save read date to robot_data[param2]
		//might want to add conditions for param1=240 and param1=50 explicitly
		//then throw ROS_ERROR if param1 is none of those
		if (10==param1)
		{
			updateRobotData(param2);
		} else {
			setParameterData(param1, param2);
		}	
		
		ros::spinOnce(); //check if callbacks have happened
		
		if ((serial_fast_buffer.size()==0) && publish_encoder_vals)
		{	
			publishEncoders();
		} else if ((serial_medium_buffer.size()==0) && publish_med_rate_vals) {
			publishMedRateData();			
		} else if ((serial_slow_buffer.size()==0) && publish_slow_rate_vals) {
			publishSlowRateData();
		}
	}
	
}

void OpenRover::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("cmd_vel said: %f, %f", msg->linear.x, msg->angular.z);
    int left_motor_speed, right_motor_speed, flipper_motor_speed;
    left_motor_speed = (int)((msg->linear.x*30) + (msg->angular.z*20) + 125)%250;
    right_motor_speed =(int)((msg->linear.x*30) - (msg->angular.z*20) + 125)%250;
    flipper_motor_speed = (int)((msg->angular.y*20) + 125)%250;
	
	//Add most recent motor values to motor_speeds[3] class variable
    updateMotorSpeeds(left_motor_speed, right_motor_speed, flipper_motor_speed);
}

void OpenRover::toggleLowSpeedMode(const std_msgs::Bool::ConstPtr& msg)
{
	bool mode_setting = msg->data; //getParameterData(i_to_computer_REG_MOTOR_SLOW_SPEED);
	
	//ROS_INFO("Entered low speed toggling %i", mode_setting);
	
	if(mode_setting==false)
	{
		ROS_INFO("Switching off low speed mode.");
		serial_slow_buffer.push_back(240);
		serial_slow_buffer.push_back(0);
	}
	if(mode_setting==true)
	{
		ROS_INFO("Switching on low speed mode.");
		serial_slow_buffer.push_back(240);
		serial_slow_buffer.push_back(1);
	} 
}

void OpenRover::updateAllRobotData()
{
	int num_fast_params = sizeof(ROBOT_DATA_INDEX_FAST)/sizeof(ROBOT_DATA_INDEX_FAST[0]);
	int num_medium_params = sizeof(ROBOT_DATA_INDEX_MEDIUM)/sizeof(ROBOT_DATA_INDEX_MEDIUM[0]);
	int num_slow_params = sizeof(ROBOT_DATA_INDEX_SLOW)/sizeof(ROBOT_DATA_INDEX_SLOW[0]);
		
	//ROS_INFO("num_slow_params: %i", num_slow_params);
	for(int i = 0; i<num_slow_params; i++)
		updateRobotData(ROBOT_DATA_INDEX_SLOW[i]);

	//ROS_INFO("num_medium_params: %i", num_medium_params);
	for(int i = 0; i<num_medium_params; i++)
		updateRobotData(ROBOT_DATA_INDEX_MEDIUM[i]);
		
	
	//ROS_INFO("num_fast_params: %i", num_fast_params);
	for(int i = 0; i<num_fast_params; i++)
		updateRobotData(ROBOT_DATA_INDEX_FAST[i]);	
}

void OpenRover::updateRobotData(int param)
{
	int val = getParameterData(param);
	//check if val is good (not negative) and if not, try again
	if (0 > val)
	{		
		ROS_WARN("Trying to getParam %i again", param);
		//val = getParameterData(param);
		serial_fast_buffer.push_back(10);
		serial_fast_buffer.push_back(param);		
	}
	
	robot_data[param] = val;
	//ROS_INFO("Updated param %i to %i", param, val);
}

void OpenRover::updateMotorSpeeds(int left_motor, int right_motor, int flipper_motor)
{
	motor_speeds[0] = left_motor;
	motor_speeds[1] = right_motor;
	motor_speeds[2] = flipper_motor;
	//ROS_INFO("Updated motor speeds %i, %i, %i", left_motor, right_motor, flipper_motor);
}

bool OpenRover::sendCommand(int param1, int param2)
{
	unsigned char write_buffer[7];
	unsigned int value;
	
    write_buffer[0] = START_BYTE;
    write_buffer[1] = (char)motor_speeds[0]; //left motor
    write_buffer[2] = (char)motor_speeds[1]; //right motor
    write_buffer[3] = (char)motor_speeds[2]; //flipper
    write_buffer[4] = (char)param1; //Param 1: 10 to get data, 240 for low speed mode 
    write_buffer[5] = (char)param2; //Param 2:
    //Calculate Checksum
    write_buffer[6] = (char) 255-(write_buffer[1]+write_buffer[2]+write_buffer[3]+write_buffer[4]+write_buffer[5])%255;
	
	//ROS_INFO("Sending command %i,%i,%i,%i,%i,%i,%i", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);

	if (write(fd, write_buffer, 7)<7)
	{
		ROS_ERROR("Failed to send command %02x,%02x,%02x,%02x,%02x,%02x,%02x", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);
		return false;
	}
	//ROS_INFO("Sent command %02x,%02x,%02x,%02x,%02x,%02x,%02x", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);
	return true;
}

int OpenRover::readCommand() //must be used after a send command (param1)
{
	char read_buffer[5];
	int value, checksum;
	int bits_read = read(fd, read_buffer, 5);
	
	if(!(START_BYTE==read_buffer[0]))
	{		
		ROS_ERROR("Incorrect start byte: %i", read_buffer[0]);
		ROS_ERROR("Read_buff: %02x, %02x, %02x, %02x, %02x, %i", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4], bits_read);
		return -1;		
	}
	checksum = 255-(read_buffer[1]+read_buffer[2]+read_buffer[3])%255;
	if(!(checksum==read_buffer[4]))
	{		
		ROS_ERROR("Received bad CRC, check: %i, rec: %i", checksum, read_buffer[4]);
		ROS_ERROR("Read_buff: %02x, %02x, %02x, %02x, %02x, %i", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4], bits_read);
		return -1;
	}
	//ROS_INFO("Read_buff: %02x, %02x, %02x, %02x, %02x, %i", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4], bits_read);
		
	value = (read_buffer[2]<<8) + read_buffer[3];
	return value;
}

bool OpenRover::setParameterData(int param1, int param2)
{
	//Sometimes reads 00 so flush buffers prior to using.
	tcflush(fd, TCIOFLUSH);
	if(!sendCommand(param1, param2))
	{		
		ROS_ERROR("Failed sendCommand while setting parameter %i to %i", param1, param2);
		return false;
	}
	
	return true;
}

int OpenRover::getParameterData(int param)
{
	int value;
	
	//Sometimes reads 00 so flush buffers prior to using.
	tcflush(fd, TCIOFLUSH);
	if(!sendCommand(10, param))
	{
		ROS_ERROR("Failed sendCommand while getting parameter %i", param);
		return -1;
	}
	//Sometimes reads 00 so flush buffers prior to using.
	tcflush(fd, TCIOFLUSH);
	
	value = readCommand();
	
	if(0>value)
	{
		ROS_ERROR("Failed readCommand while getting parameter %i", param);
		return -1;
	}
	//ROS_INFO("getParameterData returned %i", value);
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
    fd_options.c_cc[VMIN] = 0x05;
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


        ros::Rate loop_rate(LOOP_RATE);

        while(ros::ok())
        {
            //encoder_pub.publish(openrover->readEncoders());
            //battery_soc_pub.publish(openrover->readBattrySOC());
            ros::spinOnce();
            openrover->SerialManager();
            //loop_rate.sleep();
        }

        delete openrover;
        delete nh_priv;
        delete nh;

        return 0;
}

