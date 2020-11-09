#pragma once

namespace openrover
{

//special build numbers that have significance for operation of the driver
const int BUILD_NUMBER_WITH_GOOD_RPM_DATA = 10007;

//empirically tuned - tested extensively
const float MOTOR_RPM_TO_MPS_RATIO = 13749 / 1.26;
const float MOTOR_RPM_TO_MPS_CFB = -0.07;
const float WHEEL_TO_TRACK_RATIO = 1.818;

const unsigned char SERIAL_START_BYTE = 253;
const int SERIAL_OUT_PACKAGE_LENGTH = 7;
const int SERIAL_IN_PACKAGE_LENGTH = 5;
const int LOOP_RATE = 1000;  // microseconds between serial manager calls

// flipper constants__________
// odometry flipper constants
const float ODOM_ENCODER_COEF_F = 110.8;                                   // r_track*2*pi*(2.54/100)/96*1e6/45
const float ODOM_AXLE_TRACK_F = 10.75;                                     // distance between centerlines of tracks
const float ODOM_ANGULAR_COEF_F = 0.5 / (ODOM_AXLE_TRACK_F * 2.54 / 100);  // rad per meter
const float ODOM_TRACTION_FACTOR_F = 0.75;
// high speed mode cmd_vel to motor command flipper constants
const int MOTOR_SPEED_LINEAR_COEF_F_HS = 90;
const int MOTOR_SPEED_ANGULAR_COEF_F_HS = 50;

// 4wd constants__________
// odometry 4wd constants
const float ODOM_ENCODER_COEF_4WD = 182.405;                                   // r_wheel*2*pi*(2.54/100)/96*1e6/45
const float ODOM_AXLE_TRACK_4WD = 14.375;                                      // distance between wheels
const float ODOM_ANGULAR_COEF_4WD = 1.0 / (ODOM_AXLE_TRACK_4WD * 2.54 / 100);  // rad per meter
const float ODOM_TRACTION_FACTOR_4WD = 0.9;
// high speed cmd_vel to motor command 4wd constants
const int MOTOR_SPEED_LINEAR_COEF_4WD_HS = 31;
const int MOTOR_SPEED_ANGULAR_COEF_4WD_HS = 6;
const float MOTOR_SPEED_CW_TURN_COEF = 1.0;

// 2wd constants__________
// odometry 2wd constants
const float ODOM_ENCODER_COEF_2WD = 182.405;                                   // r_track*2*pi*(2.54/100)/96*1e6/45
const float ODOM_AXLE_TRACK_2WD = 14.375;                                      // distance between wheels
const float ODOM_ANGULAR_COEF_2WD = 1.0 / (ODOM_AXLE_TRACK_2WD * 2.54 / 100);  // rad per meter
const float ODOM_TRACTION_FACTOR_2WD = 0.9877;
// high speed cmd_vel to motor command 2wd constants
const int MOTOR_SPEED_LINEAR_COEF_2WD_HS = 30;
const int MOTOR_SPEED_ANGULAR_COEF_2WD_HS = 10;

// Velocity Controller Constants
const int CONTROLLER_DEADBAND_COMP = 0;  // reduce MOTOR_DEADBAND by this amount
const int MAX_ACCEL_CUTOFF = 20;         // m/s^2

// general openrover_driver platform constants
// the rovers can only go fast enough to get an encoder reading down to ~50 so
// anything lower is most likely a stall condition being misread. Over 4000 is
// a rover moving slower than ~1mm/s which is essentially stopped.
const int ENCODER_MAX = 14000;
const int ENCODER_MIN = 40;
const int MOTOR_FLIPPER_COEF = 100;

const int MOTOR_DEADBAND = 9;
const int MOTOR_NEUTRAL = 125;
const int MOTOR_SPEED_MAX = 250;
const int MOTOR_SPEED_MIN = 0;
const int MOTOR_DIFF_MAX = 200;  // Max command difference between left and

// Firmware parameters. Kept numbering from Firmware SDK-Protocol Documents_07-03-2018_1
// to maintain consistency. Some parameters are still a work in progress (WIP)
// as labeled below and so will return -1.

const int i_REG_PWR_TOTAL_CURRENT = 0;         // 5hz
const int i_REG_MOTOR_FB_RPM_LEFT = 2;         // 5hz ------ WIP
const int i_REG_MOTOR_FB_RPM_RIGHT = 4;        // 5hz ------ WIP
const int i_REG_FLIPPER_FB_POSITION_POT1 = 6;  // 5hz
const int i_REG_FLIPPER_FB_POSITION_POT2 = 8;  // 5hz

const int i_REG_MOTOR_FB_CURRENT_LEFT = 10;   // 5hz
const int i_REG_MOTOR_FB_CURRENT_RIGHT = 12;  // 5hz
const int i_REG_MOTOR_FAULT_FLAG_LEFT = 18;   // 1hz ------ WIP
const int i_REG_MOTOR_TEMP_LEFT = 20;         // 1hz
const int i_REG_MOTOR_TEMP_RIGHT = 22;        // 1hz

const int i_REG_POWER_BAT_VOLTAGE_A = 24;         // 1hz
const int i_REG_POWER_BAT_VOLTAGE_B = 26;         // 1hz
const int i_ENCODER_INTERVAL_MOTOR_LEFT = 28;     // 10hz
const int i_ENCODER_INTERVAL_MOTOR_RIGHT = 30;    // 10hz
const int i_ENCODER_INTERVAL_MOTOR_FLIPPER = 32;  // 10hz ------ WIP

const int i_REG_ROBOT_REL_SOC_A = 34;      // 1hz ------ WIP
const int i_REG_ROBOT_REL_SOC_B = 36;      // 1hz
const int i_REG_MOTOR_CHARGER_STATE = 38;  // 5hz
const int i_BUILDNO = 40;                  // 1hz
const int i_REG_POWER_A_CURRENT = 42;      // 5hz

const int i_REG_POWER_B_CURRENT = 44;      // 5hz
const int i_REG_MOTOR_FLIPPER_ANGLE = 46;  // 5hz

// const int i_to_computer_REG_MOTOR_SIDE_FAN_SPEED = 48; //5hz----WIP
// const int i_to_computer_REG_MOTOR_SLOW_SPEED = 50; //5hz ----WIP

const int i_BATTERY_STATUS_A = 52;
const int i_BATTERY_STATUS_B = 54;
const int i_BATTERY_MODE_A = 56;
const int i_BATTERY_MODE_B = 58;
const int i_BATTERY_TEMP_A = 60;
const int i_BATTERY_TEMP_B = 62;

const int i_BATTERY_VOLTAGE_A = 64;
const int i_BATTERY_VOLTAGE_B = 66;
const int i_BATTERY_CURRENT_A = 68;
const int i_BATTERY_CURRENT_B = 70;

const int ROBOT_DATA_INDEX_FAST[] = {
  i_ENCODER_INTERVAL_MOTOR_LEFT, i_ENCODER_INTERVAL_MOTOR_RIGHT,
  //, i_ENCODER_INTERVAL_MOTOR_FLIPPER  ----WIP //10hz
  i_REG_MOTOR_FB_RPM_LEFT, i_REG_MOTOR_FB_RPM_RIGHT
};

const int ROBOT_DATA_INDEX_MEDIUM[] = {
  i_REG_PWR_TOTAL_CURRENT,
  // i_REG_MOTOR_FB_RPM_LEFT,i_REG_MOTOR_FB_RPM_RIGHT, ----WIP
  i_REG_FLIPPER_FB_POSITION_POT1, i_REG_FLIPPER_FB_POSITION_POT2, i_REG_MOTOR_FB_CURRENT_LEFT,
  i_REG_MOTOR_FB_CURRENT_RIGHT, i_REG_MOTOR_CHARGER_STATE, i_REG_POWER_A_CURRENT, i_REG_POWER_B_CURRENT,
  i_REG_MOTOR_FLIPPER_ANGLE, i_BATTERY_CURRENT_A, i_BATTERY_CURRENT_B
  // i_to_computer_REG_MOTOR_SIDE_FAN_SPEED, i_to_computer_REG_MOTOR_SLOW_SPEED ----WIP
};

const int ROBOT_DATA_INDEX_SLOW[] = {
  // i_REG_MOTOR_FAULT_FLAG_LEFT, ----WIP
  i_REG_MOTOR_TEMP_LEFT,     i_REG_MOTOR_TEMP_RIGHT, i_REG_POWER_BAT_VOLTAGE_A,
  i_REG_POWER_BAT_VOLTAGE_B, i_REG_ROBOT_REL_SOC_A,  i_REG_ROBOT_REL_SOC_B,
  i_BATTERY_STATUS_A,        i_BATTERY_STATUS_B,     i_BATTERY_MODE_A,
  i_BATTERY_MODE_B,          i_BATTERY_TEMP_A,       i_BATTERY_TEMP_B,
  i_BATTERY_VOLTAGE_A,       i_BATTERY_VOLTAGE_B,    i_BUILDNO
};

const int FAST_SIZE = sizeof(ROBOT_DATA_INDEX_FAST) / sizeof(ROBOT_DATA_INDEX_FAST[0]);
const int MEDIUM_SIZE = sizeof(ROBOT_DATA_INDEX_MEDIUM) / sizeof(ROBOT_DATA_INDEX_MEDIUM[0]);
const int SLOW_SIZE = sizeof(ROBOT_DATA_INDEX_SLOW) / sizeof(ROBOT_DATA_INDEX_SLOW[0]);

struct PidGains
{
  double Kp;
  double Ki;
  double Kd;

  PidGains(float p, float i, float d) : Kp(p), Ki(i), Kd(d)
  {
  }

};

}