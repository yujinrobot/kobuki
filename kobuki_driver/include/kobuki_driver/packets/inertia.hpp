/**
 * @file /include/kobuki_driver/packets/inertia.hpp
 *
 * @brief Inertia packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_INERTIA_DATA_HPP__
#define KOBUKI_INERTIA_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class Inertia : public packet_handler::payloadBase
{
public:
  struct Data {
    int16_t angle;
    int16_t angle_rate;
    unsigned char acc[3];
  } data;

  virtual ~Inertia() {};

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_inertia: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 7;
    buildBytes(Header::Inertia, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.angle, byteStream);
    buildBytes(data.angle_rate, byteStream);
    buildBytes(data.acc[0], byteStream);
    buildBytes(data.acc[1], byteStream);
    buildBytes(data.acc[2], byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_inertia: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id, length;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);
    buildVariable(data.angle, byteStream);
    buildVariable(data.angle_rate, byteStream);
    buildVariable(data.acc[0], byteStream);
    buildVariable(data.acc[1], byteStream);
    buildVariable(data.acc[2], byteStream);

    return true;
  }
};

} // namespace kobuki

#endif /* KOBUKI_INERTIA_DATA_HPP__ */

