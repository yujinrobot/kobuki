/**
 * @file /kobuki_driver/src/driver/core_sensors.cpp
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

#include "../../include/kobuki_driver/core_sensors.hpp"

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
  buildBytes(data.remote, byteStream);
  buildBytes(data.charger, byteStream);
  buildBytes(data.battery, byteStream);

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
  buildVariable(data.remote, byteStream);
  buildVariable(data.charger, byteStream);
  buildVariable(data.battery, byteStream);

  return true;
}



} // namespace kobuki
