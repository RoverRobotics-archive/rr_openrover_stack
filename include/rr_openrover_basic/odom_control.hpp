#ifndef _odom_control_hpp
#define _odom_control_hpp

#include <cmath>
#include <vector>
#include <string>


namespace openrover {

class OdomControl
{
public:
    OdomControl(bool use_control, double Kp, double Ki, double Kd, char max, char min, std::string log_filename); //max min values for returned value
    OdomControl(bool use_control, double Kp, double Ki, double Kd, char max, char min); //max min values for returned value
    OdomControl();

    char calculate(double linear_velocity_, double measured_velocity_, double dt); //in m/s

    const char MOTOR_NEUTRAL_; // 125
    const char MOTOR_MAX_; // 250
    const char MOTOR_MIN_; // 0
    const char MOTOR_DEADBAND_;// = 9;

    const double MAX_ACCEL_CUTOFF_; //20
    const double MIN_VELOCITY_; //0.04
    const double MAX_VELOCITY_; //2.5ish?

    bool enable_file_logging_;
    bool use_control_;

    //Can poll these values to see if motor speed is saturating
    bool at_max_motor_speed_;
    bool at_min_motor_speed_;

    std::string log_filename;

    //.csv Debuggin
    std::ofstream fs_;

    //General Class variables
    double K_P_;
    double K_I_;
    double K_D_;
    double integral_value_;

    //Returned value
    char motor_speed_; //char value between 0 and 250
    char deadband_offset_;

    //velocity feedback
    double velocity_commanded_;
    double velocity_measured_;
    double velocity_filtered_;
    std::vector<double> velocity_history_;
    bool velocity_control_on_;
    double error_;
    bool skip_measurement_;
    
private:
    //Controller Functions
    void velocityController();
    double filter(double left_motor_vel, double dt);
    bool hasZeroHistory(const std::vector<double>& vel_history);
    int boundMotorSpeed(int motor_speed, int max, int min);
    char deadbandOffset(char motor_speed, char deadband_offset);
    double P(double error, double dt);
    double I(double error, double dt);
    double D(double error, double dt);
    char PID(double error, double dt);
};

}
#endif /* _odom_control_hpp */
