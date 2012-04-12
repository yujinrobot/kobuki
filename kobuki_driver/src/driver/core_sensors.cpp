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

#include "../../include/kobuki_driver/modules/core_sensors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool CoreSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (!(byteStream.size() > 0))
  {
    //ROS_WARN_STREAM("kobuki_node: kobuki_default: serialise failed. empty byte stream.");
    return false;
  }

  buildBytes(data.header_id, byteStream);
  buildBytes(data.time_stamp, byteStream);
  buildBytes(data.bump, byteStream);
  buildBytes(data.wheel_drop, byteStream);
  buildBytes(data.cliff, byteStream);
  buildBytes(data.left_encoder, byteStream);
  buildBytes(data.right_encoder, byteStream);
  buildBytes(data.left_pwm, byteStream);
  buildBytes(data.right_pwm, byteStream);
  buildBytes(data.buttons, byteStream);
  buildBytes(data.charger, byteStream);
  buildBytes(data.battery, byteStream);

  return true;
}
bool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (!(byteStream.size() > 0))
  {
    //ROS_WARN_STREAM("kobuki_node: kobuki_default: deserialise failed. empty byte stream.");
    return false;
  }

  buildVariable(data.header_id, byteStream);
  buildVariable(data.time_stamp, byteStream);
  buildVariable(data.bump, byteStream);
  buildVariable(data.wheel_drop, byteStream);
  buildVariable(data.cliff, byteStream);
  buildVariable(data.left_encoder, byteStream);
  buildVariable(data.right_encoder, byteStream);
  buildVariable(data.left_pwm, byteStream);
  buildVariable(data.right_pwm, byteStream);
  buildVariable(data.buttons, byteStream);
  buildVariable(data.charger, byteStream);
  buildVariable(data.battery, byteStream);

  return true;
}



} // namespace kobuki
