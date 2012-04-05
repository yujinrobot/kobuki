#ifndef KOBUKI_MAGNET_DATA_HPP__
#define KOBUKI_MAGNET_DATA_HPP__

#include <ecl/containers.hpp>
#include "payload_base.hpp"
#include <kobuki_comms/Header.h>
#include <kobuki_comms/Magnet.h>

<<<<<<< HEAD
namespace kobuki
{

class MagnetData : public packet_handler::payloadBase
{
public:
  // container
  kobuki_comms::Magnet data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_manget: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(data.header_id, byteStream);
    buildBytes(data.extra, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_manget: deserialise failed. empty byte stream.");
      return false;
    }

    buildVariable(data.header_id, byteStream);
    buildVariable(data.extra, byteStream);

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

class MagnetData : public packet_handler::payloadBase
{
public:
	// container
	kobuki_comms::Magnet data;
	
	// methods
	bool serialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: kobuki_manget: serialise failed. empty byte stream."); 
			return false; 
		}

		buildBytes( data.header_id,		byteStream );
		buildBytes( data.extra,			byteStream );
		return true;
	}

	bool deserialise( ecl::PushAndPop<unsigned char> & byteStream )
	{
		if(!(byteStream.size()>0)) { 
			ROS_WARN_STREAM("kobuki_node: kobuki_manget: deserialise failed. empty byte stream."); 
			return false; 
		}

		buildVariable( data.header_id, 	byteStream );
		buildVariable( data.extra, 		byteStream );

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

#endif /* KOBUKI_MAGNET_DATA_HPP__ */

