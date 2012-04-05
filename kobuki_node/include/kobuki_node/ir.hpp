#ifndef KOBUKI_IR_DATA_HPP__
#define KOBUKI_IR_DATA_HPP__

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>
#include <kobuki_comms/Header.h>
#include <kobuki_comms/IR.h>

namespace kobuki {

class IRData : public packet_handler::payloadBase
{
public:
	// container
	kobuki_comms::IR data;
	
	IRData() 
	{
		data.obstacle.resize(3);
	}
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: iclebo_ir: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header_id, 		byteStream );
		buildBytes( data.obstacle[0], byteStream );
		buildBytes( data.obstacle[1], byteStream );
		buildBytes( data.obstacle[2], byteStream );
		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: iclebo_ir: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header_id, 		byteStream );
		buildVariable( data.obstacle[0], byteStream );
		buildVariable( data.obstacle[1], byteStream );
		buildVariable( data.obstacle[2], byteStream );

		//showMe();
		return constrain();
	}

	bool constrain()
	{
		return true;
	}

	void showMe()
	{
		//printf("--[%02x || %03d | %03d | %03d]\n", data.bump, obstacle[2], obstacle[1], obstacle[0] );
	}
};

} // namespace kobuki

#endif /* KOBUKI_IR_DATA_HPP__ */
