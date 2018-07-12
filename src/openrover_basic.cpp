#include "ros/ros.h"
#include <fcntl.h>
#include <termios.h>
#include <ctime>
#include <vector>
#include <string>
#include <cmath>

#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>
#include "nav_msgs/Odometry.h"
#include <rr_openrover_basic/RawRrOpenroverBasicFastRateData.h>
#include <rr_openrover_basic/RawRrOpenroverBasicMedRateData.h>
#include <rr_openrover_basic/RawRrOpenroverBasicSlowRateData.h>

#include "rr_openrover_basic/openrover.hpp"

const int SERIAL_START_BYTE = 253;
const int SERIAL_OUT_PACKAGE_LENGTH = 7;
const int SERIAL_IN_PACKAGE_LENGTH = 5;
const int LOOP_RATE = 1000; //microseconds between serial manager calls

const float ENCODER_COEF = 104.12;
const float ODOM_SMOOTHING = 50.0;
const int MOTOR_FLIPPER_COEF = 100;
const int MOTOR_NEUTRAL = 125;
const int MOTOR_SPEED_MAX = 250;
const int MOTOR_SPEED_MIN = 0;
const int MOTOR_LINEAR_COEF = 495;
const int MOTOR_ANGULAR_COEF = 644;
const int MOTOR_DIFF_MAX = 200; //Max command difference between left and
// right motors in low speed mode, prevents overcurrent
const float MOTOR_ANGULAR_RATE_MAX = 0.155; //max turn rate in rad/s

//Firmware parameters. Kept numbering from Firmware SDK-Protocol Documents_07-03-2018_1
//to maintain consistency. Some parameters are still a work in progress (WIP)
//as labeled below and so will return -1.

const int i_REG_PWR_TOTAL_CURRENT = 0;  //5hz
const int i_REG_MOTOR_FB_RPM_LEFT = 2; //5hz ------ WIP
const int i_REG_MOTOR_FB_RPM_RIGHT = 4; //5hz ------ WIP
const int i_REG_FLIPPER_FB_POSITION_POT1 = 6; //5hz
const int i_REG_FLIPPER_FB_POSITION_POT2 = 8; //5hz 

const int i_REG_MOTOR_FB_CURRENT_LEFT = 10; //5hz
const int i_REG_MOTOR_FB_CURRENT_RIGHT = 12; //5hz
const int i_REG_MOTOR_FAULT_FLAG_LEFT = 18; //1hz ------ WIP
const int i_REG_MOTOR_TEMP_LEFT = 20; //1hz
const int i_REG_MOTOR_TEMP_RIGHT = 22; //1hz

const int i_REG_POWER_BAT_VOLTAGE_A = 24; //1hz
const int i_REG_POWER_BAT_VOLTAGE_B = 26; //1hz
const int i_ENCODER_INTERVAL_MOTOR_LEFT = 28; //10hz
const int i_ENCODER_INTERVAL_MOTOR_RIGHT = 30; //10hz
const int i_ENCODER_INTERVAL_MOTOR_FLIPPER = 32; //10hz ------ WIP

const int i_REG_ROBOT_REL_SOC_A = 34; //1hz ------ WIP
const int i_REG_ROBOT_REL_SOC_B = 36; //1hz
const int i_REG_MOTOR_CHARGER_STATE = 38;  //5hz
const int i_BUILDNO = 40;  //1hz
const int i_REG_POWER_A_CURRENT = 42;  //5hz

const int i_REG_POWER_B_CURRENT = 44; //5hz
const int i_REG_MOTOR_FLIPPER_ANGLE = 46;  //5hz

//const int i_to_computer_REG_MOTOR_SIDE_FAN_SPEED = 48; //5hz----WIP
//const int i_to_computer_REG_MOTOR_SLOW_SPEED = 50; //5hz ----WIP

const int ROBOT_DATA_INDEX_FAST[] = {
i_ENCODER_INTERVAL_MOTOR_LEFT, i_ENCODER_INTERVAL_MOTOR_RIGHT}; /*,
i_ENCODER_INTERVAL_MOTOR_FLIPPER}; ----WIP //10hz*/ 

const int ROBOT_DATA_INDEX_MEDIUM[] = {
i_REG_PWR_TOTAL_CURRENT, //i_REG_MOTOR_FB_RPM_LEFT,i_REG_MOTOR_FB_RPM_RIGHT, ----WIP
i_REG_FLIPPER_FB_POSITION_POT1, i_REG_FLIPPER_FB_POSITION_POT2,
i_REG_MOTOR_FB_CURRENT_LEFT, i_REG_MOTOR_FB_CURRENT_RIGHT, i_REG_MOTOR_CHARGER_STATE,
i_REG_POWER_A_CURRENT, i_REG_POWER_B_CURRENT, i_REG_MOTOR_FLIPPER_ANGLE
//i_to_computer_REG_MOTOR_SIDE_FAN_SPEED, i_to_computer_REG_MOTOR_SLOW_SPEED ----WIP
};

const int ROBOT_DATA_INDEX_SLOW[] = {
//i_REG_MOTOR_FAULT_FLAG_LEFT, ----WIP
i_REG_MOTOR_TEMP_LEFT,
i_REG_MOTOR_TEMP_RIGHT, i_REG_POWER_BAT_VOLTAGE_A, i_REG_POWER_BAT_VOLTAGE_B,
i_REG_ROBOT_REL_SOC_A, i_REG_ROBOT_REL_SOC_B, i_BUILDNO};

const int FAST_SIZE = sizeof(ROBOT_DATA_INDEX_FAST)/sizeof(ROBOT_DATA_INDEX_FAST[0]);
const int MEDIUM_SIZE = sizeof(ROBOT_DATA_INDEX_MEDIUM)/sizeof(ROBOT_DATA_INDEX_MEDIUM[0]);
const int SLOW_SIZE = sizeof(ROBOT_DATA_INDEX_SLOW)/sizeof(ROBOT_DATA_INDEX_SLOW[0]);

OpenRover::OpenRover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
    port_("/dev/ttyUSB0"),
    baud_(57600),
    fast_rate_(10.0), //10Hz Total Serial data is limited to 66 msgs/second
    medium_rate_(2.0), //2Hz
    slow_rate_(1.0), //1Hz
    motor_speeds_commanded_{MOTOR_NEUTRAL,MOTOR_NEUTRAL,MOTOR_NEUTRAL}, //default motor commands to neutral
    timeout_(0.5), //in seconds
    publish_fast_rate_vals_(false),
    publish_med_rate_vals_(false),
    publish_slow_rate_vals_(false),
    low_speed_mode_on_(true)
{
    ROS_INFO( "Initializing" );
    nh_priv.param( "port", port_, (std::string)"/dev/ttyUSB0" );
    nh_priv.param( "baud", baud_, 57600 );
    
    serial_fast_buffer_.reserve(10*FAST_SIZE); //reserve space for 5 sets of FAST rate data
    serial_medium_buffer_.reserve(10*MEDIUM_SIZE); //reserve space for 5 sets of Medium rate data
    serial_slow_buffer_.reserve(10*SLOW_SIZE); //reserve space for 5 sets of Slow rate data
    
    //WallTimers simplify the timing of updating parameters by reloading serial buffers at specified rates.
    //without them the serial buffers will never be loaded with new commands
    fast_timer = nh.createWallTimer( ros::WallDuration(1.0/fast_rate_), &OpenRover::robotDataFastCB, this);
    medium_timer = nh.createWallTimer( ros::WallDuration(1.0/medium_rate_), &OpenRover::robotDataMediumCB, this);
    slow_timer = nh.createWallTimer( ros::WallDuration(1.0/slow_rate_), &OpenRover::robotDataSlowCB, this);
    timeout_timer = nh.createWallTimer( ros::WallDuration(timeout_), &OpenRover::timeoutCB, this, true);
}

bool OpenRover::start()
{
    if (!(nh.getParam("/openrover_basic_node/port", port_)))
    {
        ROS_WARN("Failed to retrieve port from parameter server.");
    }
    openComs();
    ROS_INFO("Creating Publishers and Subscribers");
    ROS_INFO("Fast Data List: %i, Med Data List: %i, Slow Data List: %i", FAST_SIZE, MEDIUM_SIZE, SLOW_SIZE);
    ROS_INFO("Number of messages per sec (must be less than 66): %i", FAST_SIZE*10+MEDIUM_SIZE*2+SLOW_SIZE*1);
    fast_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicFastRateData>("rr_openrover_basic/raw_fast_rate_data",1);
    medium_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicMedRateData>("rr_openrover_basic/raw_med_rate_data",1);
    slow_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicSlowRateData>("rr_openrover_basic/raw_slow_rate_data",1);
    
    cmd_vel_sub = nh.subscribe("/cmd_vel/managed", 1, &OpenRover::cmdVelCB, this);
    
    if (!(nh.getParam("/openrover_basic_node/timeout", timeout_)))
    {
        ROS_WARN("Failed to retrieve timeout from parameter server.", timeout_);
    }
        
    if (!(nh.getParam("/openrover_basic_node/drive_type", drive_type_)))
    {
        ROS_WARN("Failed to retrieve drive_type from parameter server.");
    }
    
    if (!(nh.getParam("/openrover_basic_node/default_low_speed_mode", low_speed_mode_on_)))
    {
        ROS_WARN("Failed to retrieve default_low_speed_mode from parameter server.");       
    }
    
    if (low_speed_mode_on_)
    {       
        setParameterData(240, 1); //turn low speed on to keep robot from running away
        ROS_INFO("low_speed_mode: on");
    }
    else
    {       
        setParameterData(240, 0); //turn low speed on to keep robot from running away
        ROS_INFO("low_speed_mode: off");        
    }
    return true;
}

void OpenRover::robotDataSlowCB(const ros::WallTimerEvent &e)
{
    for(int i = 0; i<SLOW_SIZE; i++)
    {
        serial_slow_buffer_.push_back(10);
        serial_slow_buffer_.push_back(ROBOT_DATA_INDEX_SLOW[i]);
    }
    publish_slow_rate_vals_ = true;
}

void OpenRover::robotDataMediumCB(const ros::WallTimerEvent &e)
{
    for(int i = 0; i<MEDIUM_SIZE; i++)
    {
        serial_medium_buffer_.push_back(10);
        serial_medium_buffer_.push_back(ROBOT_DATA_INDEX_MEDIUM[i]);
    }
    publish_med_rate_vals_ = true;
}

void OpenRover::robotDataFastCB(const ros::WallTimerEvent &e)
{   
    for(int i = 0; i<FAST_SIZE; i++)
    {
        serial_fast_buffer_.push_back(10);
        serial_fast_buffer_.push_back(ROBOT_DATA_INDEX_FAST[i]);
    }
    publish_fast_rate_vals_ = true;
}

void OpenRover::timeoutCB(const ros::WallTimerEvent &e)
{
    updateMotorSpeedsCommanded(MOTOR_NEUTRAL, MOTOR_NEUTRAL, MOTOR_NEUTRAL);    
}

void OpenRover::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{    
    int left_motor_speed, right_motor_speed, flipper_motor_speed;
    float turn_rate = msg->angular.z;
    float linear_rate = msg->linear.x;    
    
    timeout_timer.stop();
    right_motor_speed = (int)((linear_rate*MOTOR_LINEAR_COEF) + (turn_rate*MOTOR_ANGULAR_COEF) + 125);
    left_motor_speed = (int)((linear_rate*MOTOR_LINEAR_COEF) - (turn_rate*MOTOR_ANGULAR_COEF) + 125);
    flipper_motor_speed = (int)((msg->angular.y*MOTOR_FLIPPER_COEF) + 125)%250;
    if (right_motor_speed > MOTOR_SPEED_MAX)
    {
        right_motor_speed = MOTOR_SPEED_MAX;
    }
    if (left_motor_speed > MOTOR_SPEED_MAX)
    {
        left_motor_speed = MOTOR_SPEED_MAX;
    }
    if (right_motor_speed < MOTOR_SPEED_MIN)
    {
        right_motor_speed = MOTOR_SPEED_MIN;
    }
    if (left_motor_speed < MOTOR_SPEED_MIN)
    {
        left_motor_speed = MOTOR_SPEED_MIN;
    }
    //The following reduces max difference between motor commands to prevent over current in flippers
    if ((right_motor_speed-left_motor_speed)>MOTOR_DIFF_MAX)
    {
        int average_motor_speed = (right_motor_speed + left_motor_speed)/2;
        right_motor_speed = average_motor_speed + MOTOR_DIFF_MAX/2;
        left_motor_speed = average_motor_speed - MOTOR_DIFF_MAX/2;
    }
    if ((left_motor_speed-right_motor_speed)>MOTOR_DIFF_MAX)
    {
        int average_motor_speed = (left_motor_speed-right_motor_speed)/2;
        left_motor_speed = average_motor_speed + MOTOR_DIFF_MAX/2;
        right_motor_speed = average_motor_speed - MOTOR_DIFF_MAX/2;
    }

    //Add most recent motor values to motor_speeds_commanded_[3] class variable
    updateMotorSpeedsCommanded((char)left_motor_speed, (char)right_motor_speed, (char)flipper_motor_speed);
    timeout_timer.start();
}

void OpenRover::publishFastRateData()
{
    static float left_odom = 0;
    static float right_odom = 0;
    
    rr_openrover_basic::RawRrOpenroverBasicFastRateData msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    
    msg.left_motor = robot_data_[i_ENCODER_INTERVAL_MOTOR_LEFT];
    msg.right_motor = robot_data_[i_ENCODER_INTERVAL_MOTOR_RIGHT];
    msg.flipper_motor = robot_data_[i_ENCODER_INTERVAL_MOTOR_FLIPPER];
    fast_rate_pub.publish(msg);
    publish_fast_rate_vals_ = false;
}

void OpenRover::publishMedRateData()
{
    rr_openrover_basic::RawRrOpenroverBasicMedRateData med_msg;
    
    med_msg.header.stamp = ros::Time::now();
    med_msg.header.frame_id = "";
    
    med_msg.reg_pwr_total_current = robot_data_[i_REG_PWR_TOTAL_CURRENT];
    med_msg.reg_motor_fb_rpm_left = robot_data_[i_REG_MOTOR_FB_RPM_LEFT];
    med_msg.reg_motor_fb_rpm_right = robot_data_[i_REG_MOTOR_FB_RPM_RIGHT]; 
    med_msg.reg_flipper_fb_position_pot1 = robot_data_[i_REG_FLIPPER_FB_POSITION_POT1];
    med_msg.reg_flipper_fb_position_pot2 = robot_data_[i_REG_FLIPPER_FB_POSITION_POT2];
    med_msg.reg_motor_fb_current_left = robot_data_[i_REG_MOTOR_FB_CURRENT_LEFT];   
    med_msg.reg_motor_fb_current_right = robot_data_[i_REG_MOTOR_FB_CURRENT_RIGHT];
    med_msg.reg_motor_charger_state = robot_data_[i_REG_MOTOR_CHARGER_STATE];
    med_msg.reg_power_a_current = robot_data_[i_REG_POWER_A_CURRENT];   
    med_msg.reg_power_b_current = robot_data_[i_REG_POWER_B_CURRENT];
    med_msg.reg_motor_flipper_angle = robot_data_[i_REG_MOTOR_FLIPPER_ANGLE];
    
    medium_rate_pub.publish(med_msg);
    publish_med_rate_vals_ = false;
}

void OpenRover::publishSlowRateData()
{
    rr_openrover_basic::RawRrOpenroverBasicSlowRateData slow_msg;
    
    slow_msg.header.stamp = ros::Time::now();
    slow_msg.header.frame_id = "";
    
    slow_msg.reg_motor_fault_flag_left = robot_data_[i_REG_MOTOR_FAULT_FLAG_LEFT];
    slow_msg.reg_motor_temp_left = robot_data_[i_REG_MOTOR_TEMP_LEFT];
    slow_msg.reg_motor_temp_right = robot_data_[i_REG_MOTOR_TEMP_RIGHT];    
    slow_msg.reg_power_bat_voltage_a = robot_data_[i_REG_POWER_BAT_VOLTAGE_A];
    slow_msg.reg_power_bat_voltage_b = robot_data_[i_REG_POWER_BAT_VOLTAGE_B];
    slow_msg.reg_robot_rel_soc_a = robot_data_[i_REG_ROBOT_REL_SOC_A];      
    slow_msg.reg_robot_rel_soc_b = robot_data_[i_REG_ROBOT_REL_SOC_B];  
    slow_msg.buildno = robot_data_[i_BUILDNO];
    
    slow_rate_pub.publish(slow_msg);
    publish_slow_rate_vals_ = false;
}

//sends serial commands stored in the 3 buffers in order of speed with fast getting highest priority
void OpenRover::serialManager()
{
    char param1;
    char param2;
    while ((serial_fast_buffer_.size()>1) || (serial_medium_buffer_.size()>1) || (serial_slow_buffer_.size()>1))
    {
        if (serial_fast_buffer_.size()>1)
        {   
            param2 = serial_fast_buffer_.back();
            serial_fast_buffer_.pop_back();
            
            param1 = serial_fast_buffer_.back();
            serial_fast_buffer_.pop_back();         
            
        }
        else if (serial_medium_buffer_.size()>1)
        {
            
            param2 = serial_medium_buffer_.back();
            serial_medium_buffer_.pop_back();
            
            param1 = serial_medium_buffer_.back();
            serial_medium_buffer_.pop_back();
            
        }
        else if (serial_slow_buffer_.size()>1)
        {
            
            param2 = serial_slow_buffer_.back();
            serial_slow_buffer_.pop_back();
            
            param1 = serial_slow_buffer_.back();
            serial_slow_buffer_.pop_back();
            
        }
        else
        {
            param2 = 0;
            param1 = 0; 
        }
        
        //Check callbacks in case any new motor commands or timer callbacks were triggered
        // since entering the Serial Manager
        ros::spinOnce();
        
        //Check param1 to determine what communication to the robot is required
        try{
            if (param1==10) // Param1==10 requests data with index of param2
            {
                updateRobotData(param2);
            }
            else if (param1==20)
            { //param1==20 means sending fan speed of param2
                setParameterData(param1, param2);               
            }
            else if (param1==240)
            { //param1==240 means sending low speed command
                setParameterData(param1, param2);               
            }
            else if (param1==250)
            { //param1==250 means calibrating flipper DO NOT USE OFTEN
                setParameterData(param1, param2);               
            }
            else if (param1==0)
            { //param1==0 means buffers are empty and doesn't need to do anything
            }
            else
            {
                throw std::string("Unknown param1. Removing parameter from buffer");
            }
        }
        catch (std::string s)
        {
            throw;
        }
        catch (...)
        {
            throw;
        }
        
        //If one of the buffers are empty, publish the values
        if ((serial_fast_buffer_.size()==0) && publish_fast_rate_vals_)
        {   
            publishFastRateData();
        }
        else if ((serial_medium_buffer_.size()==0) && publish_med_rate_vals_)
        {
            publishMedRateData();           
        }
        else if ((serial_slow_buffer_.size()==0) && publish_slow_rate_vals_)
        {
            publishSlowRateData();
        }
    }
}

void OpenRover::updateRobotData(int param)
{
    try
    {
        int data = getParameterData(param);
        if (0 > data) //check if val is good (not negative) and if not, push param back to buffer
        {       
            throw;
        }
        
        robot_data_[param] = data;
    }
    catch(std::string s)
    {
        char str_ex [50];
        sprintf(str_ex, "Failed to update param %i. ", param);
        throw std::string(str_ex) + s;
    }
}

void OpenRover::updateMotorSpeedsCommanded(char left_motor, char right_motor, char flipper_motor)
{ //updates the stored motor speeds to the most recent commanded motor speeds
    motor_speeds_commanded_[0] = left_motor;
    motor_speeds_commanded_[1] = right_motor;
    motor_speeds_commanded_[2] = flipper_motor;
}

bool OpenRover::sendCommand(int param1, int param2)
{
    unsigned char write_buffer[SERIAL_OUT_PACKAGE_LENGTH];
    
    write_buffer[0] = SERIAL_START_BYTE;
    write_buffer[1] = (char)motor_speeds_commanded_[0]; //left motor
    write_buffer[2] = (char)motor_speeds_commanded_[1]; //right motor
    write_buffer[3] = (char)motor_speeds_commanded_[2]; //flipper
    write_buffer[4] = (char)param1; //Param 1: 10 to get data, 240 for low speed mode 
    write_buffer[5] = (char)param2; //Param 2:
    //Calculate Checksum
    write_buffer[6] = (char) 255-(write_buffer[1]+write_buffer[2]+write_buffer[3]+write_buffer[4]+write_buffer[5])%255;
    
    if (write(fd, write_buffer, SERIAL_OUT_PACKAGE_LENGTH)<SERIAL_OUT_PACKAGE_LENGTH)
    {
        char str_ex [50];
        sprintf(str_ex, "Failed to send command: %02x,%02x,%02x,%02x,%02x,%02x,%02x", write_buffer[0],write_buffer[1],write_buffer[2],write_buffer[3],write_buffer[4],write_buffer[5],write_buffer[6]);
        throw std::string(str_ex);
    }
    return true;
}

int OpenRover::readCommand() //only used after a send command with param1==10
{
    char read_buffer[SERIAL_IN_PACKAGE_LENGTH];
    int data, checksum;
    int bits_read = read(fd, read_buffer, SERIAL_IN_PACKAGE_LENGTH);
    
    if(!(SERIAL_START_BYTE==read_buffer[0]))
    {
        char str_ex [50];
        sprintf(str_ex, "Received bad start byte. Received: %02x,%02x,%02x,%02x,%02x", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
        throw std::string(str_ex);
    }
    checksum = 255-(read_buffer[1]+read_buffer[2]+read_buffer[3])%255;
    if(!(checksum==read_buffer[4]))
    {
        char str_ex [50];
        sprintf(str_ex, "Received bad CRC. Received: %02x,%02x,%02x,%02x,%02x", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
        throw std::string(str_ex);
    }

    data = (read_buffer[2]<<8) + read_buffer[3];
    return data;
}

bool OpenRover::setParameterData(int param1, int param2)
{
    try
    {
        if(!sendCommand(param1, param2))
        {       
            throw;
        }
        
        return true;
    }
    catch (std::string s)
    {
        std::string s2("setParamaterData() failed. ");
        throw (s2 + s);
    }
}

int OpenRover::getParameterData(int param)
{
    int data;
    
    try
    {
        if(!sendCommand(10, param))
        {
            throw;
        }
        
        data = readCommand();
        
        if(0>data)
        {
            throw;
        }
        return data;
    }
    catch(std::string s)
    {
        std::string s2("getParameterData() failed. ");// %i. ", param);
        throw (s2 + s);
    }
}

bool OpenRover::openComs()
{
    ROS_INFO("Opening serial port");
    struct termios fd_options;

    fd = ::open( port_.c_str( ), O_RDWR | O_NOCTTY | O_NDELAY );
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
    fd_options.c_iflag &= ~( IUCLC | IXANY | IMAXBEL | IXON | IXOFF | IUTF8 | ICRNL | INPCK ); //input modes
    fd_options.c_oflag |= ( NL0 | CR0 | TAB0 | BS0 | VT0 | FF0 );
    fd_options.c_oflag &= ~( OPOST | ONLCR | OLCUC | OCRNL | ONOCR | ONLRET | OFILL | OFDEL | NL1 | CR1 | CR2 | TAB3 | BS1 | VT1 | FF1 );
    fd_options.c_lflag |= ( NOFLSH );
    fd_options.c_lflag &= ~( ICANON | IEXTEN | TOSTOP | ISIG | ECHOPRT | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE );
    fd_options.c_cc[VINTR] = 0x03;  //INTR Character
    fd_options.c_cc[VQUIT] = 0x1C;  //QUIT Character
    fd_options.c_cc[VERASE] = 0x7F; //ERASE Character
    fd_options.c_cc[VKILL] = 0x15;  //KILL Character
    fd_options.c_cc[VEOF] = 0x04; //EOF Character
    fd_options.c_cc[VTIME] = 0x01; //Timeout in 0.1s of serial read
    fd_options.c_cc[VMIN] = SERIAL_IN_PACKAGE_LENGTH; //Min Number of bytes to read
    fd_options.c_cc[VSWTC] = 0x00;
    fd_options.c_cc[VSTART] = SERIAL_START_BYTE;  //START Character
    fd_options.c_cc[VSTOP] = 0x13;  //STOP character
    fd_options.c_cc[VSUSP] = 0x1A;  //SUSP character
    fd_options.c_cc[VEOL] = 0x00;  //EOL Character
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
            //Check callbacks
            try
            {
                ros::spinOnce();
                //Process Serial Buffers
                openrover->serialManager();
                loop_rate.sleep();
            }
            catch(std::string s)
            {
                ROS_ERROR(s.c_str());
            }
            catch(...)
            {
                ROS_ERROR("Unknown Exception occurred");
            }
        }

        delete openrover;
        delete nh_priv;
        delete nh;

        return 0;
}
