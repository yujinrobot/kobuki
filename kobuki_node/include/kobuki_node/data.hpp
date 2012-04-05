#ifndef KOBUKI_DATA_HPP__
#define KOBUKI_DATA_HPP__

#include <ecl/containers.hpp>
#include <kobuki_comms/SensorData.h>
#include <packet_handler/payload_base.hpp>

namespace kobuki {

//1//
class Data : public packet_handler::payloadBase
{
public:
	// group
	unsigned char header0;
	unsigned short int time_stamp;
	unsigned char bump;
	unsigned char wheel_drop;
	unsigned char cliff;
	unsigned short int encoder[2];	// left and right
	unsigned char pwm[2];			// left and right
	unsigned char dustbin;			// 1:locked 0:unlocked
	unsigned char remote;
	unsigned char button;
	unsigned char charger; 			//
	unsigned char battery;			// [mA/10]
	unsigned char caster;			// tick
	unsigned char over_current;		// wheel left/right, brush, side-brush, vacuum

	// group 1
	unsigned char header1;
	unsigned char obstacle[3];
	unsigned char docking;

	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { ROS_WARN_STREAM("kobuki_node: iclebo_data: serialise failed. empty byte stream."); return false; }
		buildBytes( header0, 	byteStream );
		buildBytes( time_stamp, 	byteStream );
		buildBytes( bump, 		byteStream );
		buildBytes( wheel_drop, 	byteStream );
		buildBytes( cliff, 		byteStream );
		buildBytes( encoder[0], 	byteStream );
		buildBytes( encoder[1], 	byteStream );
		buildBytes( pwm[0], 		byteStream );
		buildBytes( pwm[1], 		byteStream );
		buildBytes( dustbin, 	byteStream );
		buildBytes( remote, 		byteStream );
//		buildBytes( button, byteStream );
		buildBytes( charger, 	byteStream );
		buildBytes( battery, 	byteStream );
		buildBytes( caster, 		byteStream );
		buildBytes( over_current,byteStream );

		buildBytes( header1, 	byteStream );
		buildBytes( obstacle[0], byteStream );
		buildBytes( obstacle[1], byteStream );
		buildBytes( obstacle[2], byteStream );
		buildBytes( docking, 	byteStream );

		return true;
	}
	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { ROS_WARN_STREAM("kobuki_node: iclebo_data: deserialise failed. empty byte stream."); return false; }
		buildVariable( header0, 	byteStream );
		buildVariable( time_stamp, 	byteStream );
		buildVariable( bump, 		byteStream );
		buildVariable( wheel_drop, 	byteStream );
		buildVariable( cliff, 		byteStream );
		buildVariable( encoder[0], 	byteStream );
		buildVariable( encoder[1], 	byteStream );
		buildVariable( pwm[0], 		byteStream );
		buildVariable( pwm[1], 		byteStream );
		buildVariable( dustbin, 	byteStream );
		buildVariable( remote, 		byteStream );
//		buildVariable( button, byteStream );
		buildVariable( charger, 	byteStream );
		buildVariable( battery, 	byteStream );
		buildVariable( caster, 		byteStream );
		buildVariable( over_current,byteStream );

		buildVariable( header1, 	byteStream );
		buildVariable( obstacle[0], byteStream );
		buildVariable( obstacle[1], byteStream );
		buildVariable( obstacle[2], byteStream );
		buildVariable( docking, 	byteStream );

		return constrain();
	}

	bool constrain()
	{
		return true;
	}

	void showMe()
	{
		printf("--[%02x || %03d | %03d | %03d]\n", bump, obstacle[2], obstacle[1], obstacle[0] );
	}
};

//2//
class Data2 : public packet_handler::payloadBase
{
public:
	// group
	kobuki_comms::SensorData data;

	// group 1
	unsigned char header1;
	unsigned char obstacle[3];
	unsigned char docking;

	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { ROS_WARN_STREAM("kobuki_node: iclebo_data2: serialise failed. empty byte stream."); return false; }
		buildBytes( data.header0, 		byteStream );
		buildBytes( data.time_stamp, 	byteStream );
		buildBytes( data.bump, 			byteStream );
		buildBytes( data.wheel_drop, 	byteStream );
		buildBytes( data.cliff, 		byteStream );
		buildBytes( data.left_encoder, 	byteStream );
		buildBytes( data.right_encoder, byteStream );
		buildBytes( data.left_pwm, 		byteStream );
		buildBytes( data.right_pwm, 	byteStream );
		buildBytes( data.dustbin, 		byteStream );
		buildBytes( data.remote, 		byteStream );
//		buildBytes( button, 			byteStream );
		buildBytes( data.charger, 		byteStream );
		buildBytes( data.battery, 		byteStream );
		buildBytes( data.caster, 		byteStream );
		buildBytes( data.over_current,	byteStream );

		buildBytes( header1, 	 byteStream );
		buildBytes( obstacle[0], byteStream );
		buildBytes( obstacle[1], byteStream );
		buildBytes( obstacle[2], byteStream );
		buildBytes( docking, 	 byteStream );

		return true;
	}
	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { ROS_WARN_STREAM("kobuki_node: iclebo_data2: deserialise failed. empty byte stream."); return false; }
		buildVariable( data.header0, 		byteStream );
		buildVariable( data.time_stamp,		byteStream );
		buildVariable( data.bump, 			byteStream );
		buildVariable( data.wheel_drop, 	byteStream );
		buildVariable( data.cliff, 			byteStream );
		buildVariable( data.left_encoder, 	byteStream );
		buildVariable( data.right_encoder, 	byteStream );
		buildVariable( data.left_pwm, 		byteStream );
		buildVariable( data.right_pwm, 		byteStream );
		buildVariable( data.dustbin, 		byteStream );
		buildVariable( data.remote, 		byteStream );
//		buildVariable( data.button, 		byteStream );
		buildVariable( data.charger, 		byteStream );
		buildVariable( data.battery, 		byteStream );
		buildVariable( data.caster, 		byteStream );
		buildVariable( data.over_current,	byteStream );

		buildVariable( header1, 	byteStream );
		buildVariable( obstacle[0], byteStream );
		buildVariable( obstacle[1], byteStream );
		buildVariable( obstacle[2], byteStream );
		buildVariable( docking, 	byteStream );

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
		printf("--[%02x || %03d | %03d | %03d][ %03d| %03d]\n", data.bump, obstacle[2], obstacle[1], obstacle[0], data.left_encoder, data.right_encoder );
	}
};

} // namespace kobuki

#endif /* KOBUKI_DATA_HPP__ */
