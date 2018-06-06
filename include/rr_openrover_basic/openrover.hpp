#ifndef _openrover_hpp
#define _openrover_hpp

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <stdint.h>
#include <boost/thread.hpp>
#include <string>


class openrover
{
public:
	openrover( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );
	bool open( );
	void close( );
	bool start( );
	void stop( );

	void JointStateCB( const ros::WallTimerEvent &e );

private:
	bool is_open( ) const;
	void CmdVelCB( const geometry_msgs::TwistPtr &msg );

	// functions for sending information to the kangaroo
	bool send_get_request(unsigned char address, char channel, unsigned char desired_parameter);
	bool set_channel_speed(double speed, unsigned char address, char channel);
	bool send_start_signals(uint8_t address);

	// functions used for the request - response  (JointStateCB)
	int get_parameter(unsigned char address, char channel, unsigned char desired_parameter);
	int read_message(unsigned char address, bool& ok);
	uint8_t read_one_byte(bool& ok);
	int evaluate_kangaroo_response( uint8_t address, uint8_t* header, uint8_t* data, bool& ok);
	void handle_errors(uint8_t address, int error_code);

	// address of the serial port
	std::string port;
	// the number of lines of the encoder
	int encoder_lines_per_revolution;
	// the hertz that the JointState will be published at
	int hz;

	// file descriptor, which is used for accessing serial ports in C.
	//   it's essentially an address to the serial port
	int fd;
	// Node handles
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::WallTimer poll_timer;
	ros::Subscriber joint_traj_sub;
	ros::Publisher joint_state_pub;
	
	// mutex-es for accessing the serial that the kangaroo is connected on
	// output_mutex *must* be locked first
	boost::mutex output_mutex;
	boost::mutex input_mutex;

};

//}

#endif /* _openrover_hpp */
