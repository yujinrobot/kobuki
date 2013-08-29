/**
 * @file /include/kobuki_driver/packets/hardware.hpp
 *
 * @brief Hardware version request packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_HW_DATA_HPP__
#define KOBUKI_HW_DATA_HPP__

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

class Hardware : public packet_handler::payloadBase
{
public:
  Hardware() : packet_handler::payloadBase(true, 2) {};
  struct Data {
    uint32_t version;
  } data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    unsigned char length = 4;
    buildBytes(Header::Hardware, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.version, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (byteStream.size() < length+2)
    {
      //std::cout << "kobuki_node: kobuki_hw: deserialise failed. not enough byte stream." << std::endl;
      return false;
    }

    unsigned char header_id, length_packed;
    buildVariable(header_id, byteStream);
    buildVariable(length_packed, byteStream);
    if( header_id != Header::Hardware ) return false;
    if( length_packed != 2 and length_packed != 4) return false;

    // TODO First 3 firmware versions coded version number on 2 bytes, so we need convert manually to our new
    // 4 bytes system; remove this horrible, dirty hack as soon as we upgrade the firmware to 1.1.2 or 1.2.0
    if (length_packed == 2)
    {
      uint16_t old_style_version = 0;
      buildVariable(old_style_version, byteStream);

      if (old_style_version == 104)
        data.version = 0x00010004;//65540; // 1.0.4
    }
    else
    {
      buildVariable(data.version, byteStream);
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
  }
};

} // namespace kobuki

#endif /* KOBUKI_HW_DATA_HPP__ */

