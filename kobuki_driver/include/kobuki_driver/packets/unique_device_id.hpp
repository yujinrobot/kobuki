/**
 * @file /include/kobuki_driver/packets/unique_device_id.hpp
 * @author Younghun Ju <younghoon.ju@rnd.yujinrobot.com> <yhju83@gmail.com>
 * @brief Module for handling of unique device id request packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_UDID_DATA_HPP__
#define KOBUKI_UDID_DATA_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class UniqueDeviceID : public packet_handler::payloadBase
{
public:
  struct Data {
    uint32_t udid0;
    uint32_t udid1;
    uint32_t udid2;
  } data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_udid: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 12;
    buildBytes(Header::UniqueDeviceID, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.udid0, byteStream);
    buildBytes(data.udid1, byteStream);
    buildBytes(data.udid2, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_udid: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id, length;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);
    buildVariable(data.udid0, byteStream);
    buildVariable(data.udid1, byteStream);
    buildVariable(data.udid2, byteStream);

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
};

} // namespace kobuki

#endif /* KOBUKI_UDID_DATA_HPP__ */

