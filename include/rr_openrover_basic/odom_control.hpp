#ifndef _odom_control_hpp
#define _odom_control_hpp

#include <cmath>
#include <vector>
#include <string>


namespace openrover {

class OdomControl
{
public:
    OdomControl(bool use_control, double Kp, double Ki,
        double Kd, char max, char min, std::string log_filename); //max min values for returned value
    OdomControl(bool use_control, double Kp, double Ki,
        double Kd, char max, char min); //, std::string log_filename); //max min values for returned value

    char calculate(double linear_velocity_, double measured_velocity_, double dt); //in m/s

    const char MOTOR_NEUTRAL_; // 125
    const char MOTOR_MAX_; // 250
    const char MOTOR_MIN_; // 0
    const char MOTOR_DEADBAND_;// = 9;

    const float MAX_ACCEL_CUTOFF_; //20
    const float MIN_VELOCITY_; //0.04
    const float MAX_VELOCITY_; //2.5ish?

    bool enable_file_logging_;
    bool use_control_;

    //Can poll these values to see if motor speed is saturating
    bool at_max_motor_speed_;
    bool at_min_motor_speed_;

    std::string log_filename;

private:
    //.csv Debuggin
    std::ofstream fs_;

    //General Class variables
    float K_P_;
    float K_I_;
    float K_D_;

    //Returned value
    char motor_speed_; //char value between 0 and 250

    //velocity feedback
    float velocity_commanded_;
    float velocity_measured_;
    float velocity_filtered_;
    std::vector<float> velocity_history_;
    bool velocity_control_on_;
    float error_;
    bool skip_measurement_;
    
    //Controller Functions
    void velocityController();
    void filter(float left_motor_vel, float dt);
    bool hasZeroHistory(const std::vector<float>& vel_history);
    int boundMotorSpeed(int motor_speed, int max, int min);
};

}
#endif /* _odom_control_hpp */
