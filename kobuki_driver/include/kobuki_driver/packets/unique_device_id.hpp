/**
 * @file /include/kobuki_driver/packets/unique_device_id.hpp
 * @author Younghun Ju <yhju@yujinrobot.com> <yhju83@gmail.com>
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
  UniqueDeviceID() : packet_handler::payloadBase(false, 12) {};

  struct Data {
    uint32_t udid0;
    uint32_t udid1;
    uint32_t udid2;
  } data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    buildBytes(Header::UniqueDeviceID, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.udid0, byteStream);
    buildBytes(data.udid1, byteStream);
    buildBytes(data.udid2, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (byteStream.size() < length+2)
    {
      //std::cout << "kobuki_node: kobuki_udid: deserialise failed. not enough byte stream." << std::endl;
      return false;
    }

    unsigned char header_id, length_packed;
    buildVariable(header_id, byteStream);
    buildVariable(length_packed, byteStream);
    if( header_id != Header::UniqueDeviceID ) return false;
    if( length_packed != length ) return false;

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
  }
};

} // namespace kobuki

#endif /* KOBUKI_UDID_DATA_HPP__ */

