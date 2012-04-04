#ifndef __ICLEBO_CURRENT_DATA_HPP__
#define __ICLEBO_CURRENT_DATA_HPP__

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>
#include <iclebo_comms/iCleboHeader.h>
#include <iclebo_comms/iCleboCurrent.h>

namespace iclebo {

class iCleboCurrentData : public packet_handler::payloadBase
{
public:
	// container
	iclebo_comms::iCleboCurrent data;
	
	iCleboCurrentData() 
	{
		data.current.resize(2);
	}
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_current: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header_id, 		byteStream );
		buildBytes( data.current[0], byteStream );
		buildBytes( data.current[1], byteStream );
		//buildBytes( data.current[2], byteStream );
		//buildBytes( data.current[3], byteStream );
		//buildBytes( data.current[4], byteStream );
		//buildBytes( data.current[5], byteStream );
		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_current: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header_id, 		byteStream );
		buildVariable( data.current[0], byteStream );
		buildVariable( data.current[1], byteStream );
		//buildVariable( data.current[2], byteStream );
		//buildVariable( data.current[3], byteStream );
		//buildVariable( data.current[4], byteStream );
		//buildVariable( data.current[5], byteStream );

		//showMe();
		return constrain();
	}

	bool constrain()
	{
		return true;
	}

	void showMe()
	{
		//printf("--[%02x || %03d | %03d | %03d]\n", data.bump, current[2], current[1], current[0] );
	}
};

} // namespace iclebo

#endif /* __ICLEBO_CURRENT_DATA_HPP__ */
