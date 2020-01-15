#pragma once

#include <cmath>
#include <vector>
#include <string>

#include "rr_openrover_driver/constants.hpp"

namespace openrover
{
class OdomControl
{
public:
  OdomControl();  // default

  OdomControl(bool use_control, PidGains pid_gains, int max, int min,
              std::ofstream* fs);                                // max min values for returned value
  OdomControl(bool use_control, PidGains pid_gains, int max, int min);  // max min values for returned value

  unsigned char run(bool e_stop_on, bool control_on, double commanded_vel, double measured_vel,
                    double dt);  // in m/s
  void start(bool use_control, PidGains pid_gains, int max, int min);
  void reset();

  int MOTOR_MAX_;       // 250
  int MOTOR_MIN_;       // 0
  int MOTOR_DEADBAND_;  // = 9;

  double MAX_ACCEL_CUTOFF_;  // 20
  double MIN_VELOCITY_;      // 0.04
  double MAX_VELOCITY_;      // 2.5ish?

  bool enable_file_logging_;
  bool use_control_;

  // Can poll these values to see if motor speed is saturating
  bool at_max_motor_speed_;
  bool at_min_motor_speed_;
  bool stop_integrating_;

  //.csv Debuggin
  std::ofstream* fs_;

  // General Class variables
  double K_P_;
  double K_I_;
  double K_D_;
  double integral_value_;
  double velocity_error_;

  // Returned value
  int motor_speed_;  // value between 0-250
  unsigned char deadband_offset_;

  // velocity feedback
  double velocity_commanded_;
  double velocity_measured_;
  double velocity_filtered_;
  std::vector<double> velocity_history_;
  bool velocity_control_on_;
  bool skip_measurement_;

private:
  // Controller Functions
  void velocityController();
  double filter(double left_motor_vel, double dt);
  bool hasZeroHistory(const std::vector<double>& vel_history);
  int boundMotorSpeed(int motor_speed, int max, int min);
  int deadbandOffset(int motor_speed, int deadband_offset);
  double P(double error, double dt);
  double I(double error, double dt);
  double D(double error, double dt);
  int PID(double error, double dt);
  int feedThroughControl();
};

}