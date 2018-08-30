#include "ros/ros.h"
#include <fcntl.h>
#include <termios.h>
#include <ctime>
#include <vector>
#include <string>
#include <cmath>

#include "tf/tf.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/TwistStamped.h"
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

//flipper constants__________
//odometry flipper constants
const float ODOM_ENCODER_COEF_F = 110.8; //r_track*2*pi*(2.54/100)/96*1e6/45
const float ODOM_AXLE_TRACK_F  = 10.75; //distance between centerlines of tracks
const float ODOM_ANGULAR_COEF_F = 0.5/(ODOM_AXLE_TRACK_F*2.54/100); //rad per meter
const float ODOM_SLIPPAGE_FACTOR_F = 0.75;
//low speed mode cmd_vel to motor command flipper constants
const int MOTOR_SPEED_LINEAR_COEF_F_LS = 495;
const int MOTOR_SPEED_ANGULAR_COEF_F_LS = 644;
//high speed mode cmd_vel to motor command flipper constants
const int MOTOR_SPEED_LINEAR_COEF_F_HS = 495;
const int MOTOR_SPEED_ANGULAR_COEF_F_HS = 644;

//4wd constants__________
//odometry 4wd constants
const float ODOM_ENCODER_COEF_4WD = 182.405; //r_wheel*2*pi*(2.54/100)/96*1e6/45
const float ODOM_AXLE_TRACK_4WD  = 14.375; //distance between wheels
const float ODOM_ANGULAR_COEF_4WD = 1.0/(ODOM_AXLE_TRACK_4WD*2.54/100); //rad per meter
const float ODOM_SLIPPAGE_FACTOR_4WD = 0.610;
// low speed mode cmd_vel to motor command 4wd constants
const int MOTOR_SPEED_LINEAR_COEF_4WD_LS = 293;
const int MOTOR_SPEED_ANGULAR_COEF_4WD_LS = 86;
//high speed cmd_vel to motor command 4wd constants
const int MOTOR_SPEED_LINEAR_COEF_4WD_HS = 31;
const int MOTOR_SPEED_ANGULAR_COEF_4WD_HS = 6;
const float MOTOR_SPEED_WEIGHT_COEF_A = 0.0034383;
const float MOTOR_SPEED_WEIGHT_COEF_B = -0.011618;
const float MOTOR_SPEED_WEIGHT_COEF_C = 0.99181;
const float MOTOR_SPEED_CW_TURN_COEF = 1.0;


//2wd constants__________
//odometry 2wd constants
const float ODOM_ENCODER_COEF_2WD = 182.405; //r_track*2*pi*(2.54/100)/96*1e6/45
const float ODOM_AXLE_TRACK_2WD  = 14.375; //distance between wheels
const float ODOM_ANGULAR_COEF_2WD = 1.0/(ODOM_AXLE_TRACK_2WD*2.54/100); //rad per meter
const float ODOM_SLIPPAGE_FACTOR_2WD = 0.9877;
//low speed mode cmd_vel to motor command 2wd constants
const int MOTOR_SPEED_LINEAR_COEF_2WD_LS = 293;
const int MOTOR_SPEED_ANGULAR_COEF_2WD_LS = 56;
//high speed cmd_vel to motor command 2wd constants
const int MOTOR_SPEED_LINEAR_COEF_2WD_HS = 293;
const int MOTOR_SPEED_ANGULAR_COEF_2WD_HS = 56;

//general openrover_basic platform constants
const int ENCODER_MAX = 5000;
const int ENCODER_MIN = 1;
const int MOTOR_FLIPPER_COEF = 100;

const float ODOM_SMOOTHING = 50.0;

const float WEIGHT_COMPENSATION_FACTOR = 10.0;
const int MOTOR_DEADBAND = 9;
const int MOTOR_NEUTRAL = 125;
const int MOTOR_SPEED_MAX = 250;
const int MOTOR_SPEED_MIN = 0;
const int MOTOR_DIFF_MAX = 200; //Max command difference between left and
// right motors in low speed mode, prevents overcurrent

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
    ROS_INFO( "Initializing openrover driver." );
    //nh_priv.param( "port", port_, (std::string)"/dev/ttyUSB0" );
    //nh_priv.param( "baud", baud_, 57600 );
    
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
    if(!setupRobotParams())
    {
        ROS_WARN("Failed to setup Robot parameters.");
    }

    ROS_INFO("Creating Publishers and Subscribers");
    ROS_INFO("Fast Data List: %i, Med Data List: %i, Slow Data List: %i", FAST_SIZE, MEDIUM_SIZE, SLOW_SIZE);
    ROS_INFO("Number of messages per sec (must be less than 66): %i", FAST_SIZE*10+MEDIUM_SIZE*2+SLOW_SIZE*1);
    fast_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicFastRateData>("rr_openrover_basic/raw_fast_rate_data",1);
    medium_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicMedRateData>("rr_openrover_basic/raw_med_rate_data",1);
    slow_rate_pub = nh.advertise<rr_openrover_basic::RawRrOpenroverBasicSlowRateData>("rr_openrover_basic/raw_slow_rate_data",1);
    odom_enc_pub = nh.advertise<nav_msgs::Odometry>("rr_openrover_basic/odom_encoder", 1);
    is_charging_pub = nh.advertise<std_msgs::Bool>("rr_openrover_basic/charging", 1);

    test_odom_cmd_vel_pub = nh.advertise<std_msgs::Int32MultiArray>("rr_openrover_basic/motor_speeds_commanded", 1);

    cmd_vel_sub = nh.subscribe("/cmd_vel/managed", 1, &OpenRover::cmdVelCB, this);
    
    return true;
}

bool OpenRover::setupRobotParams()
{//Get ROS params and save them to class variables
    if (!(nh.getParam("/openrover_basic_node/port", port_)))
    {
        ROS_WARN("Failed to retrieve port from parameter server.");
        return false;
    }

    if (!(openComs()))
    {
        ROS_WARN("Failed to start serial comunication.");
    }

    if (!(nh.getParam("/openrover_basic_node/default_low_speed_mode", low_speed_mode_on_)))
    {
        ROS_WARN("Failed to retrieve default_low_speed_mode from parameter server.");
        return false;
    }

    if (low_speed_mode_on_)
    {
        setParameterData(240, 1); //turn low speed on to keep robot from running away
        ROS_INFO("low_speed_mode: on");
    }
    else
    {
        setParameterData(240, 0);
        ROS_INFO("low_speed_mode: off");
    }
    if (!(nh.getParam("/openrover_basic_node/timeout", timeout_)))
    {
        ROS_ERROR("Failed to retrieve timeout from parameter server. Defaulting to ");
        return false;
    }
    
    if (!(nh.getParam("/openrover_basic_node/total_weight", total_weight_)))
    {
        ROS_ERROR("Failed to retrieve total_weight_ from parameter server. Defaulting to 0");
        total_weight_ = 0;
        return false;
    }

    if (!(nh.getParam("/openrover_basic_node/drive_type", drive_type_)))
    {
        ROS_ERROR("Failed to retrieve drive_type from parameter.Defaulting to flippers.");
        drive_type_ = "flippers";
        return false;
    }

    if(drive_type_==(std::string) "2wd")
    {
        ROS_INFO("2wd parameters loaded.");
        odom_encoder_coef_ = ODOM_ENCODER_COEF_2WD;
        odom_axle_track_ = ODOM_AXLE_TRACK_2WD;
        odom_angular_coef_ = ODOM_ANGULAR_COEF_2WD;
        odom_slippage_factor_ = ODOM_SLIPPAGE_FACTOR_2WD;

        if (low_speed_mode_on_)
        {
            motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_2WD_LS;
            motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_2WD_LS;
        }
        else
        {
            motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_2WD_HS;
            motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_2WD_HS;
            motor_speed_deadband_ = (int) MOTOR_DEADBAND;
            motor_speed_angular_deadband_ = (int) MOTOR_DEADBAND;
            cw_turn_coef_ = MOTOR_SPEED_CW_TURN_COEF;
        }
    }
    else if (drive_type_==(std::string) "4wd")
    {
        ROS_INFO("4wd parameters loaded.");
        odom_encoder_coef_ = ODOM_ENCODER_COEF_4WD;
        odom_axle_track_ = ODOM_AXLE_TRACK_4WD;
        odom_angular_coef_ = ODOM_ANGULAR_COEF_4WD;
        odom_slippage_factor_ = ODOM_SLIPPAGE_FACTOR_4WD;

        if (low_speed_mode_on_)
        {
            motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_4WD_LS;
            motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_4WD_LS;
            motor_speed_deadband_ = (int) MOTOR_DEADBAND;
            motor_speed_angular_deadband_ = (int) MOTOR_DEADBAND;
            cw_turn_coef_ = 1;
        }
        else
        {
            float a = MOTOR_SPEED_WEIGHT_COEF_A;
            float b = MOTOR_SPEED_WEIGHT_COEF_B;
            float c = MOTOR_SPEED_WEIGHT_COEF_C;

            weight_coef_ = a * total_weight_*total_weight_ + b * total_weight_ + c;
            //ROS_INFO("%f, %f, %f", (a * total_weight_*total_weight_), (b * total_weight_), c);
            motor_speed_linear_coef_ = (int) MOTOR_SPEED_LINEAR_COEF_4WD_HS;
            motor_speed_angular_coef_ = (int) MOTOR_SPEED_ANGULAR_COEF_4WD_HS*weight_coef_;
            motor_speed_deadband_ = (int) MOTOR_DEADBAND;
            motor_speed_angular_deadband_ = (int) MOTOR_DEADBAND*weight_coef_;
            cw_turn_coef_ = MOTOR_SPEED_CW_TURN_COEF;
            //ROS_INFO("%i,%i,%i, %f, %f", motor_speed_linear_coef_, motor_speed_angular_coef_, motor_speed_deadband_, weight_coef_, total_weight_);
        }

    }
    else if (drive_type_==(std::string) "flippers")
    {
        ROS_INFO("flipper parameters loaded.");
        odom_encoder_coef_ = ODOM_ENCODER_COEF_F;
        odom_axle_track_ = ODOM_AXLE_TRACK_F;
        odom_angular_coef_ = ODOM_ANGULAR_COEF_F;
        odom_slippage_factor_ = ODOM_SLIPPAGE_FACTOR_F;

        if (low_speed_mode_on_)
        {
            motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_F_LS;
            motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_F_LS;
            motor_speed_deadband_ = (int) MOTOR_DEADBAND;
            motor_speed_angular_deadband_ = (int) MOTOR_DEADBAND;
            cw_turn_coef_ = 1;
        }
        else
        {
            motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_F_HS;
            motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_F_HS;
            motor_speed_deadband_ = (int) MOTOR_DEADBAND;
            motor_speed_angular_deadband_ = (int) MOTOR_DEADBAND;
            cw_turn_coef_ = MOTOR_SPEED_CW_TURN_COEF;
        }
    }
    else
    {
        ROS_WARN("Unclear ROS param drive_type. Defaulting to flippers params.");
        odom_encoder_coef_ = ODOM_ENCODER_COEF_F;
        odom_axle_track_ = ODOM_AXLE_TRACK_F;
        odom_angular_coef_ = ODOM_ANGULAR_COEF_F;
        odom_slippage_factor_ = ODOM_SLIPPAGE_FACTOR_F;
        
        if (low_speed_mode_on_)
        {
            motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_F_LS;
            motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_F_LS;
            motor_speed_deadband_ = (int) MOTOR_DEADBAND;
            motor_speed_angular_deadband_ = (int) MOTOR_DEADBAND;
            cw_turn_coef_ = 1;
        }
        else
        {
            motor_speed_linear_coef_ = MOTOR_SPEED_LINEAR_COEF_F_HS;
            motor_speed_angular_coef_ = MOTOR_SPEED_ANGULAR_COEF_F_HS;
            motor_speed_deadband_ = (int) MOTOR_DEADBAND;
            motor_speed_angular_deadband_ = (int) MOTOR_DEADBAND;
            cw_turn_coef_ = MOTOR_SPEED_CW_TURN_COEF;
        }
    }

    if (!(nh.getParam("/openrover_basic_node/slippage_factor", odom_slippage_factor_)))
    {
        ROS_ERROR("Failed to retrieve odom_slippage_factor_ from parameter.Defaulting to drive_type_ slippage_factor");
        return false;
    }

    if (!(nh.getParam("/openrover_basic_node/odom_covariance_0", odom_covariance_0_)))
    {
        ROS_ERROR("Failed to retrieve odom_covariance_0 from parameter. Defaulting to 0.01");
        odom_covariance_0_ = 0.01;
        return false;
    }

    if (!(nh.getParam("/openrover_basic_node/odom_covariance_35", odom_covariance_35_)))
    {
        ROS_ERROR("Failed to retrieve odom_covariance_35 from parameter. Defaulting to 0.03");
        odom_covariance_35_ = 0.03;
        return false;
    }

    ROS_INFO("Openrover parameters loaded:");
    ROS_INFO("port: %s", port_.c_str());
    ROS_INFO("drive_type: %s", drive_type_.c_str());
    ROS_INFO("timeout: %f s", timeout_);
    ROS_INFO("default_low_speed_mode: %i", low_speed_mode_on_);
    ROS_INFO("total_weight: %f kg", total_weight_);
    ROS_INFO("slippage_factor: %f", odom_slippage_factor_);
    ROS_INFO("odom_covariance_0: %f", odom_covariance_0_);
    ROS_INFO("odom_covariance_35: %f", odom_covariance_35_);
    //ROS_INFO("enable_timeout: %i", enable_timeout)
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
{//Timer goes off when a command isn't received soon enough. Sets motors to stop values
    updateMotorSpeedsCommanded(MOTOR_NEUTRAL, MOTOR_NEUTRAL, MOTOR_NEUTRAL);
}

void OpenRover::cmdVelCB(const geometry_msgs::TwistStamped::ConstPtr& msg)
{//converts from cmd_vel (m/s and radians/s) into motor speed commands
    float left_motor_speed, right_motor_speed;
    int flipper_motor_speed;
    int motor_speed_deadband_scaled;
    float turn_rate = msg->twist.angular.z;
    float linear_rate = msg->twist.linear.x;
    float flipper_rate = msg->twist.angular.y;
    bool is_moving_forward, is_turning_cw, is_stationary, is_zero_point_turn;

    timeout_timer.stop();
    if ((linear_rate == 0) && (turn_rate != 0))
    {
        is_zero_point_turn = true;
    }
    else
    {
        is_zero_point_turn = false;
    }
    if (turn_rate > 0){
        is_turning_cw = true;
    }
    else
    {
        is_turning_cw = false;
    }

    if (is_zero_point_turn)
    {
        motor_speed_deadband_scaled = motor_speed_deadband_ * weight_coef_;
        //ROS_INFO("Is is_zero_point_turn");
        if (is_turning_cw)
        {
            motor_speed_deadband_scaled = motor_speed_deadband_ * weight_coef_ * cw_turn_coef_;
            //ROS_INFO("is_turning_cw");
        }
    }
    else
    {
        motor_speed_deadband_scaled = motor_speed_deadband_;
        //ROS_INFO("Normal Deadband");
    }
    right_motor_speed = round((linear_rate*motor_speed_linear_coef_) + (turn_rate*motor_speed_angular_coef_)) + 125;
    left_motor_speed = round((linear_rate*motor_speed_linear_coef_) - (turn_rate*motor_speed_angular_coef_)) + 125;
    //ROS_INFO("%f, %f, %i", left_motor_speed, right_motor_speed, motor_speed_deadband_scaled); //, motor_speed_angular_deadband_);
    flipper_motor_speed = ((int)round(flipper_rate*motor_speed_flipper_coef_) + 125) % 250;

    //Compensate for deadband
    if (right_motor_speed > 125)
    {
        right_motor_speed += motor_speed_deadband_scaled;
    }
    else if (right_motor_speed < 125 )
    {
        right_motor_speed -= motor_speed_deadband_scaled;
    }

    if (left_motor_speed > 125)
    {
        left_motor_speed += motor_speed_deadband_scaled;
    }
    else if (left_motor_speed < 125 )
    {
        left_motor_speed -= motor_speed_deadband_scaled;
    }

    //Bound motor speeds to be between 0-250
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
        float average_motor_speed = (right_motor_speed + left_motor_speed)/2;
        right_motor_speed = average_motor_speed + MOTOR_DIFF_MAX/2;
        left_motor_speed = average_motor_speed - MOTOR_DIFF_MAX/2;
    }
    if ((left_motor_speed-right_motor_speed)>MOTOR_DIFF_MAX)
    {
        float average_motor_speed = (left_motor_speed+right_motor_speed)/2;
        left_motor_speed = average_motor_speed + MOTOR_DIFF_MAX/2;
        right_motor_speed = average_motor_speed - MOTOR_DIFF_MAX/2;
    }

    //Add most recent motor values to motor_speeds_commanded_[3] class variable
    updateMotorSpeedsCommanded((char)round(left_motor_speed), (char)round(right_motor_speed), (char)(flipper_motor_speed));
    timeout_timer.start();
}

void OpenRover::publishOdomEnc()
{//convert encoder readings to real world values and publish as Odometry
    int left_enc = robot_data_[i_ENCODER_INTERVAL_MOTOR_LEFT];
    int right_enc = robot_data_[i_ENCODER_INTERVAL_MOTOR_RIGHT];
    
    static double left_vel = 0;
    static double right_vel = 0;
    static double left_dist = 0;
    static double right_dist = 0;
    static double pos_x = 0;
    static double pos_y = 0;
    static double theta = 0;
    double net_vel = 0;
    double diff_vel = 0;
    double alpha = 0;
    double dt = 0;
    
    tf::Quaternion q_new;
    ros::Time ros_now_time = ros::Time::now();
    double now_time = ros_now_time.toSec();
    static double past_time = 0;
    
    nav_msgs::Odometry odom_msg;
    
    dt = now_time-past_time;
    
    if((ENCODER_MIN < left_enc) && (left_enc < ENCODER_MAX))
    {
        if (motor_speeds_commanded_[0] > 125)
        {            
            left_vel = odom_encoder_coef_/left_enc;
        }
        else
        {
            left_vel = -odom_encoder_coef_/left_enc;
        }
    }
    else
    {
        left_vel = 0;
    }
    
    if((ENCODER_MIN < right_enc) && (right_enc < ENCODER_MAX))
    {
        if ( motor_speeds_commanded_[1] > 125)
        {
            right_vel = odom_encoder_coef_/right_enc;
        }
        else
        {
            right_vel = -odom_encoder_coef_/right_enc;
        }
    }
    else
    {
        right_vel = 0;
    }
    
    if(past_time!=0)
    {
        left_dist = left_dist + left_vel*dt;
        right_dist = right_dist + right_vel*dt;
            
        net_vel = 0.5*(left_vel+right_vel);
        diff_vel = right_vel - left_vel;
        
        alpha = odom_angular_coef_*diff_vel;
        
        pos_x = pos_x + net_vel*cos(theta)*dt;
        pos_y = pos_y + net_vel*sin(theta)*dt;
        theta = (theta + odom_slippage_factor_*alpha*dt);
        
        q_new = tf::createQuaternionFromRPY(0, 0, theta);
        quaternionTFToMsg(q_new, odom_msg.pose.pose.orientation);
    }
    
    odom_msg.header.stamp = ros_now_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.twist.twist.linear.x = net_vel;
    odom_msg.twist.twist.angular.z = alpha;
    
    odom_msg.twist.covariance[0] = odom_covariance_0_;
    odom_msg.twist.covariance[35] = odom_covariance_35_;

    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    
    odom_enc_pub.publish(odom_msg);
    past_time=now_time;
}

void OpenRover::publishFastRateData()
{
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
    std_msgs::Bool is_charging_msg;
    
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

    if ( robot_data_[i_REG_MOTOR_CHARGER_STATE] == 0xDADA) {
        is_charging_= true;
        is_charging_msg.data = true;
        is_charging_pub.publish(is_charging_msg);
    } 
    else
    {
        is_charging_ = false;
        is_charging_msg.data = false;
        is_charging_pub.publish(is_charging_msg);
    }
    
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
void OpenRover::publishTestOdomCmdVel()
{
    std_msgs::Int32MultiArray motor_speeds_msg;
    motor_speeds_msg.data.clear();
    //std::vector<int8_t> motor_speeds_vec (motor_speeds_commanded_, motor_speeds_commanded_ + sizeof(motor_speeds_commanded_)/sizeof(motor_speeds_commanded_[0]));
        //vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );
    /*motor_speeds_vec[0] = motor_speeds_commanded_[0]; //left
    motor_speeds_vec[1] = motor_speeds_commanded_[1]; //right
    motor_speeds_vec[2] = motor_speeds_commanded_[2]; //flipper*/
    //std::vector<int8_t> motor_speeds_vec (motor_speeds_commanded_
    motor_speeds_msg.data.push_back(motor_speeds_commanded_[0]);
    motor_speeds_msg.data.push_back(motor_speeds_commanded_[1]);
    motor_speeds_msg.data.push_back(motor_speeds_commanded_[2]);
    //motor_speeds_msg.data = motor_speeds_vec;
    test_odom_cmd_vel_pub.publish(motor_speeds_msg);
}
void OpenRover::serialManager()
{//sends serial commands stored in the 3 buffers in order of speed with fast getting highest priority
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
            publishOdomEnc();
            publishTestOdomCmdVel();
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
{//updates the stored motor speeds to the most recent commanded motor speeds
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

int OpenRover::readCommand()
{//only used after a send command with param1==10
    unsigned char read_buffer[SERIAL_IN_PACKAGE_LENGTH];
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
                ROS_ERROR("%s", s.c_str());
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
