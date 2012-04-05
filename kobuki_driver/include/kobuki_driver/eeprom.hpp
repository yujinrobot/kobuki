#ifndef KOBUKI_EEPROM_DATA_HPP__
#define KOBUKI_EEPROM_DATA_HPP__

#include <ecl/containers.hpp>
#include "payload_base.hpp"
#include <kobuki_comms/Header.h>
#include <kobuki_comms/EEPROM.h>

<<<<<<< HEAD
namespace kobuki
{

class EEPROMData : public packet_handler::payloadBase
{
public:
  // container
  kobuki_comms::EEPROM data;

  EEPROMData()
  {
    data.tmp_eeprom.resize(16);
  }

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_eeprom: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(data.header_id, byteStream);
    buildBytes(data.tmp_frame_id, byteStream);
    for (unsigned int i = 0; i < data.tmp_eeprom.size(); ++i)
    {
      buildBytes(data.tmp_eeprom[i], byteStream);
    }
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_eeprom: deserialise failed. empty byte stream.");
      return false;
    }

    buildVariable(data.header_id, byteStream);
    buildVariable(data.tmp_frame_id, byteStream);
    for (unsigned int i = 0; i < data.tmp_eeprom.size(); ++i)
    {
      buildVariable(data.tmp_eeprom[i], byteStream);
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
=======
namespace kobuki {

class EEPROMData : public packet_handler::payloadBase
{
public:
	// container
	kobuki_comms::EEPROM data;
	
	EEPROMData() 
	{
		data.tmp_eeprom.resize(16);
	}
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: kobuki_eeprom: serialise failed. empty byte stream."); 
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
			ROS_WARN_STREAM("kobuki_node: kobuki_eeprom: deserialise failed. empty byte stream."); 
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
>>>>>>> branch 'master' of git@github.com:yujinrobot/kobuki.git
};

} // namespace kobuki

#endif /* KOBUKI_EEPROM_DATA_HPP__ */

