#ifndef __ICLEBO_ST_GYRO_DATA_HPP__
#define __ICLEBO_ST_GYRO_DATA_HPP__

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>
#include <iclebo_comms/iCleboHeader.h>
#include <iclebo_comms/iCleboStGyro.h>

namespace iclebo {

class iCleboStGyroData : public packet_handler::payloadBase
{
public:
	// container
	iclebo_comms::iCleboStGyro data;
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_st_gyro: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header_id,				byteStream );
		buildBytes( data.frame_id,				byteStream );
		buildBytes( data.followed_data_length,	byteStream );
		for( unsigned int i=0; i<data.followed_data_length; ++i ) {
			buildBytes( data.st_gyro_data[i], byteStream );
		}
		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_st_gyro: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header_id,				byteStream );
		buildVariable( data.frame_id,				byteStream );
		buildVariable( data.followed_data_length,	byteStream );
		data.st_gyro_data.resize( data.followed_data_length );
		for( unsigned int i=0; i<data.followed_data_length; ++i ) {
			buildVariable( data.st_gyro_data[i], byteStream );
		}

		//showMe();
		return constrain();
	}

	bool constrain()
	{
		return true;
	}

	void showMe()
	{
		//printf("--[%02x || %03d | %03d | %03d]\n", data.bump, st_gyro_data[2], st_gyro_data[1], st_gyro_data[0] );
	}
};

} // namespace iclebo

#endif /* __ICLEBO_ST_GYRO_DATA_HPP__ */

