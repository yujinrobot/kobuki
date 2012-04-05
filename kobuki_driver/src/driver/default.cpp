/**
 * @file /kobuki_driver/src/driver/default.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 06/04/2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/default.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool DefaultData::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (!(byteStream.size() > 0))
  {
    ROS_WARN_STREAM("kobuki_node: kobuki_default: serialise failed. empty byte stream.");
    return false;
  }

  buildBytes(data.header0, byteStream);
  buildBytes(data.time_stamp, byteStream);
  buildBytes(data.bump, byteStream);
  buildBytes(data.wheel_drop, byteStream);
  buildBytes(data.cliff, byteStream);
  buildBytes(data.left_encoder, byteStream);
  buildBytes(data.right_encoder, byteStream);
  buildBytes(data.left_pwm, byteStream);
  buildBytes(data.right_pwm, byteStream);
  //buildBytes( data.dustbin,                 byteStream );
  buildBytes(data.remote, byteStream);
  //buildBytes( data.button,          byteStream );
  buildBytes(data.charger, byteStream);
  buildBytes(data.battery, byteStream);
  //buildBytes( data.caster,          byteStream );
  //buildBytes( data.over_current,    byteStream );

  return true;
}
bool DefaultData::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (!(byteStream.size() > 0))
  {
    ROS_WARN_STREAM("kobuki_node: kobuki_default: deserialise failed. empty byte stream.");
    return false;
  }

  buildVariable(data.header0, byteStream);
  buildVariable(data.time_stamp, byteStream);
  buildVariable(data.bump, byteStream);
  buildVariable(data.wheel_drop, byteStream);
  buildVariable(data.cliff, byteStream);
  buildVariable(data.left_encoder, byteStream);
  buildVariable(data.right_encoder, byteStream);
  buildVariable(data.left_pwm, byteStream);
  buildVariable(data.right_pwm, byteStream);
  //buildVariable( data.dustbin,              byteStream );
  buildVariable(data.remote, byteStream);
  //buildVariable( data.button,               byteStream );
  buildVariable(data.charger, byteStream);
  buildVariable(data.battery, byteStream);
  //buildVariable( data.caster,               byteStream );
  //buildVariable( data.over_current, byteStream );

  //showMe();
  return constrain();
}

bool DefaultData::constrain()
{
  return true;
}

void DefaultData::showMe()
{
//  std::cout << "Timestamp: " << static_cast<int>(data.time_stamp) << std::endl;
//  std::cout << "  left_encoder: " << static_cast<int>(data.left_encoder) << std::endl;
//  std::cout << "  right_encoder: " << static_cast<int>(data.right_encoder) << std::endl;
  //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, obstacle[2], obstacle[1], obstacle[0] );
}



} // namespace kobuki
