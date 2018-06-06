#include <openrover_basic/openrover_basic.hpp>

int main( int argc, char *argv[] )
{
	ros::NodeHandle *nh = NULL;
	ros::NodeHandle *nh_priv = NULL;
	kangaroo *kang = NULL;

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
	rover = new openrover( *nh, *nh_priv );
	if( !rover )
	{
		ROS_FATAL( "Failed to initialize driver" );
		delete nh_priv;
		delete nh;
		ros::shutdown( );
		return -3;
	}
	if( !rover->start( ) )
		ROS_ERROR( "Failed to start the driver" );

	ros::spin( );

	delete rover;
	delete nh_priv;
	delete nh;

	return 0;
}
