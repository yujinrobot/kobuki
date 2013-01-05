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
  struct Data {
    uint32_t version;
  } data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_hw: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 4;
    buildBytes(Header::Hardware, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.version, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_hw: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id = 0, length = 0;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);

    // TODO First 3 firmware versions coded version number on 2 bytes, so we need convert manually to our new
    // 4 bytes system; remove this horrible, dirty hack as soon as we upgrade the firmware to 1.1.2 or 1.2.0
    if (length == 2)
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
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, acc[2], acc[1], acc[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_HW_DATA_HPP__ */

