#ifndef KOBUKI_INERTIA_DATA_HPP__
#define KOBUKI_INERTIA_DATA_HPP__

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>
#include <kobuki_comms/Header.h>
#include <kobuki_comms/Inertia.h>

namespace kobuki {

class InertiaData : public packet_handler::payloadBase
{
public:
	// container
	kobuki_comms::Inertia data;
	
	InertiaData() 
	{
		data.acc.resize(3);
	}
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: iclebo_inertia: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header_id,		byteStream );
		buildBytes( data.angle,			byteStream );
		buildBytes( data.angle_rate,	byteStream );
		buildBytes( data.acc[0], byteStream );
		buildBytes( data.acc[1], byteStream );
		buildBytes( data.acc[2], byteStream );
		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: iclebo_inertia: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header_id, 	byteStream );
		buildVariable( data.angle, 		byteStream );
		buildVariable( data.angle_rate,	byteStream );
		buildVariable( data.acc[0], byteStream );
		buildVariable( data.acc[1], byteStream );
		buildVariable( data.acc[2], byteStream );

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

} // namespace kobuki

#endif /* KOBUKI_INERTIA_DATA_HPP__ */

