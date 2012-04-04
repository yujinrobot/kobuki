#ifndef __ICLEBO_TIME_DATA_HPP__
#define __ICLEBO_TIME_DATA_HPP__

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>
#include <iclebo_comms/iCleboHeader.h>
#include <iclebo_comms/iCleboTime.h>

namespace iclebo {

class iCleboTimeData : public packet_handler::payloadBase
{
public:
	// container
	iclebo_comms::iCleboTime data;
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_time: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header_id,		byteStream );
		buildBytes( data.hh,			byteStream );
		buildBytes( data.mm,			byteStream );
		buildBytes( data.ss,			byteStream );
		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_time: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header_id, 	byteStream );
		buildVariable( data.hh, 		byteStream );
		buildVariable( data.mm, 		byteStream );
		buildVariable( data.ss, 		byteStream );

		//showMe();
		return constrain();
	}

	bool constrain()
	{
		return true;
	}

	void showMe()
	{
		//printf("--[%02x || %03d | %03d | %03d]\n", data.bump, acc[2], acc[1], acc[0] );
	}
};

} // namespace iclebo

#endif /* __ICLEBO_TIME_DATA_HPP__ */

