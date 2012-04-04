#ifndef __ICLEBO_DEFAULT_DATA_HPP__
#define __ICLEBO_DEFAULT_DATA_HPP__

#include <ecl/containers.hpp>
#include <iclebo_comms/iClebo.h>
#include <packet_handler/payload_base.hpp>

namespace iclebo {

class iCleboDefaultData : public packet_handler::payloadBase
{
public:
	iclebo_comms::iClebo data;

	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_default: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header0, 		byteStream );
		buildBytes( data.time_stamp, 	byteStream );
		buildBytes( data.bump, 			byteStream );
		buildBytes( data.wheel_drop, 	byteStream );
		buildBytes( data.cliff, 		byteStream );
		buildBytes( data.left_encoder, 	byteStream );
		buildBytes( data.right_encoder, byteStream );
		buildBytes( data.left_pwm, 		byteStream );
		buildBytes( data.right_pwm, 	byteStream );
		//buildBytes( data.dustbin, 		byteStream );
		buildBytes( data.remote, 		byteStream );
		//buildBytes( data.button, 		byteStream );
		buildBytes( data.charger, 		byteStream );
		buildBytes( data.battery, 		byteStream );
		//buildBytes( data.caster, 		byteStream );
		//buildBytes( data.over_current,	byteStream );

		return true;
	}
	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("iclebo_ros_node: iclebo_default: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header0, 		byteStream );
		buildVariable( data.time_stamp,		byteStream );
		buildVariable( data.bump, 			byteStream );
		buildVariable( data.wheel_drop, 	byteStream );
		buildVariable( data.cliff, 			byteStream );
		buildVariable( data.left_encoder, 	byteStream );
		buildVariable( data.right_encoder, 	byteStream );
		buildVariable( data.left_pwm, 		byteStream );
		buildVariable( data.right_pwm, 		byteStream );
		//buildVariable( data.dustbin, 		byteStream );
		buildVariable( data.remote, 		byteStream );
		//buildVariable( data.button, 		byteStream );
		buildVariable( data.charger, 		byteStream );
		buildVariable( data.battery, 		byteStream );
		//buildVariable( data.caster, 		byteStream );
		//buildVariable( data.over_current,	byteStream );

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

} // namespace iclebo

#endif /* __ICLEBO_DEFAULT_DATA_HPP__ */
