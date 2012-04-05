#ifndef KOBUKI_COMMAND_DATA_HPP__
#define KOBUKI_COMMAND_DATA_HPP__

#include <ecl/containers.hpp>
#include "payload_base.hpp"
#include <kobuki_comms/Header.h>
#include <kobuki_comms/Command.h>

<<<<<<< HEAD
namespace kobuki
{

class CommandData : public packet_handler::payloadBase
{
public:
  // container
  kobuki_comms::Command data;

  CommandData()
  {
    //data.acc.resize(3);
  }

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_command: serialise failed. empty byte stream.");
      return false;
    }

    switch (data.command)
    {
      case kobuki_comms::Command::commandBaseControl:
        buildBytes(data.command, byteStream);
        buildBytes(data.speed, byteStream);
        buildBytes(data.radius, byteStream);
        break;
      case kobuki_comms::Command::commandCleaningMotor:
        buildBytes(data.command, byteStream);
        buildBytes(data.brush_power, byteStream);
        buildBytes(data.vacuum_power, byteStream);
        buildBytes(data.side_brush_power, byteStream);
        break;
      case kobuki_comms::Command::commandSound:
        buildBytes(data.command, byteStream);
        buildBytes(data.note, byteStream);
        buildBytes(data.duration, byteStream);
        break;
      case kobuki_comms::Command::commandSoundSequence:
        buildBytes(data.command, byteStream);
        buildBytes(data.segment_name, byteStream);
        break;
      case kobuki_comms::Command::commandAnimation:
        buildBytes(data.command, byteStream);
        buildBytes(data.animation_code, byteStream);
        buildBytes(data.options, byteStream);
        break;
      case kobuki_comms::Command::commandDisplay:
        buildBytes(data.command, byteStream);
        for (unsigned int i = 0; i > data.digits.size(); ++i)
          buildBytes(data.digits[i], byteStream);
        for (unsigned int i = 0; i > data.tmp_intensity.size(); ++i)
          buildBytes(data.tmp_intensity[i], byteStream);
        break;
      case kobuki_comms::Command::commandSetTime:
        buildBytes(data.command, byteStream);
        buildBytes(data.hh, byteStream);
        buildBytes(data.mm, byteStream);
        break;
      case kobuki_comms::Command::commandSetPower:
        buildBytes(data.command, byteStream);
        buildBytes(data.power_control_flags, byteStream);
        break;
      case kobuki_comms::Command::commandRequestExtra:
        buildBytes(data.command, byteStream);
        buildBytes(data.request_flags, byteStream);
        break;
      case kobuki_comms::Command::commandChangeFrame:
        buildBytes(data.command, byteStream);
        buildBytes(data.frame_id, byteStream);
        break;
      case kobuki_comms::Command::commandRequestEeprom:
        buildBytes(data.command, byteStream);
        buildBytes(data.frame_id, byteStream);
        break;
      case kobuki_comms::Command::commandSetDigitalOut:
        buildBytes(data.command, byteStream);
        buildBytes(data.gp_out, byteStream);
        break;
      default:
        return false;
        break;
    }

    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_command: deserialise failed. empty byte stream.");
      return false;
    }
    return true;

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
=======
namespace kobuki {

class CommandData : public packet_handler::payloadBase
{
public:
	// container
	kobuki_comms::Command data;
	
	CommandData() 
	{
		//data.acc.resize(3);
	}
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: kobuki_command: serialise failed. empty byte stream."); 
			return false; 
		}

		switch( data.command ) {
		case kobuki_comms::Command::commandBaseControl:    
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.speed,		byteStream );
			buildBytes( data.radius,	byteStream );
			break; 
		case kobuki_comms::Command::commandCleaningMotor:   
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.brush_power,		byteStream );
			buildBytes( data.vacuum_power,		byteStream );
			buildBytes( data.side_brush_power,	byteStream );
			break; 
		case kobuki_comms::Command::commandSound:           
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.note,		byteStream );
			buildBytes( data.duration,	byteStream );
			break; 
		case kobuki_comms::Command::commandSoundSequence:   
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.segment_name,		byteStream );
			break; 
		case kobuki_comms::Command::commandAnimation:       
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.animation_code,	byteStream );
			buildBytes( data.options,			byteStream );
			break; 
		case kobuki_comms::Command::commandDisplay:         
			buildBytes( data.command,    byteStream ); 
			for( unsigned int i=0; i>data.digits.size(); ++i )
				buildBytes( data.digits[i],			byteStream );
			for( unsigned int i=0; i>data.tmp_intensity.size(); ++i )
				buildBytes( data.tmp_intensity[i],	byteStream );
			break; 
		case kobuki_comms::Command::commandSetTime:        
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.hh,	byteStream );
			buildBytes( data.mm,	byteStream );
			break; 
		case kobuki_comms::Command::commandSetPower:       
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.power_control_flags,	byteStream );
			break; 
		case kobuki_comms::Command::commandRequestExtra:   
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.request_flags,	byteStream );
			break; 
		case kobuki_comms::Command::commandChangeFrame:    
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.frame_id,	byteStream );
			break; 
		case kobuki_comms::Command::commandRequestEeprom:  
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.frame_id,	byteStream );
			break; 
		case kobuki_comms::Command::commandSetDigitalOut:  
			buildBytes( data.command,    byteStream ); 
			buildBytes( data.gp_out,	byteStream );
			break; 
		default:
			return false;
			break;
		}

		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: kobuki_command: deserialise failed. empty byte stream."); 
			return false; 
		}
		return true;

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
>>>>>>> branch 'master' of git@github.com:yujinrobot/kobuki.git
};

} // namespace kobuki

#endif /* KOBUKI_COMMAND_DATA_HPP__ */

