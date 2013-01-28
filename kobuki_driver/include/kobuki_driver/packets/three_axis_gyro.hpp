/**
 * @file /include/kobuki_driver/packets/three_axis_gyro.hpp
 * @author Younghun Ju <younghoon.ju@rnd.yujinrobot.com> <yhju83@gmail.com>
 * @brief Module for handling of three_axis_gyro packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_THREE_AXIS_GYRO_DATA_HPP__
#define KOBUKI_THREE_AXIS_GYRO_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Defines
*****************************************************************************/

#define MAX_DATA_SIZE (3*8) //derived from ST_GYRO_MAX_DATA_SIZE in firmware

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class ThreeAxisGyro : public packet_handler::payloadBase
{
public:
  struct Data {
    unsigned char frame_id;
    unsigned char followed_data_length;
    unsigned short data[MAX_DATA_SIZE];
  } data;

  virtual ~ThreeAxisGyro() {};

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: three_axis_gyro: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 2 + 2 * data.followed_data_length;
    buildBytes(Header::ThreeAxisGyro, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.frame_id, byteStream);
    buildBytes(data.followed_data_length, byteStream);
    for (unsigned int i=0; i<data.followed_data_length; i++)
      buildBytes(data.data[i], byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: three_axis_gyro: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id, length;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);
    buildVariable(data.frame_id, byteStream);
    buildVariable(data.followed_data_length, byteStream);
    for (unsigned int i=0; i<data.followed_data_length; i++)
      buildVariable(data.data[i], byteStream);

    return true;
  }
};

} // namespace kobuki

#endif /* KOBUKI_THREE_AXIS_GYRO_DATA_HPP__ */

