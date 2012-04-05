#ifndef KOBUKI_EEPROM_DATA_HPP__
#define KOBUKI_EEPROM_DATA_HPP__

#include <ecl/containers.hpp>
#include <packet_handler/payload_base.hpp>
#include <iclebo_comms/iCleboHeader.h>
#include <iclebo_comms/iCleboEEPROM.h>

namespace iclebo {

class iCleboEEPROMData : public packet_handler::payloadBase
{
public:
	// container
	iclebo_comms::iCleboEEPROM data;
	
	iCleboEEPROMData() 
	{
		data.tmp_eeprom.resize(16);
	}
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: iclebo_eeprom: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header_id,		byteStream );
		buildBytes( data.tmp_frame_id,			byteStream );
		for( unsigned int i=0; i<data.tmp_eeprom.size(); ++i )
		{
			buildBytes( data.tmp_eeprom[i], byteStream );
		}
		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: iclebo_eeprom: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header_id, 	byteStream );
		buildVariable( data.tmp_frame_id, 		byteStream );
		for( unsigned int i=0; i<data.tmp_eeprom.size(); ++i )
		{
			buildVariable( data.tmp_eeprom[i], byteStream );
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
		//printf("--[%02x || %03d | %03d | %03d]\n", data.bump, tmp_eeprom[2], tmp_eeprom[1], tmp_eeprom[0] );
	}
};

} // namespace iclebo

#endif /* KOBUKI_EEPROM_DATA_HPP__ */

