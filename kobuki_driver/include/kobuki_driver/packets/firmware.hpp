/**
 * @file include/kobuki_driver/packets/firmware.hpp
 *
 * @brief Firmware version request packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_FW_DATA_HPP__
#define KOBUKI_FW_DATA_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Constants
*****************************************************************************/

#define CURRENT_FIRMWARE_MAYOR_VERSION  1
#define CURRENT_FIRMWARE_MINOR_VERSION  1
// patch number is ignored; don't need to be updated

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class Firmware : public packet_handler::payloadBase
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
      printf("kobuki_node: kobuki_fw: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 4;
    buildBytes(Header::Firmware, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.version, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_fw: deserialise failed. empty byte stream.");
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

      if (old_style_version == 123)
        data.version = 65536; // 1.0.0
      else if ((old_style_version == 10100) || (old_style_version == 110))
        data.version = 65792; // 1.1.0
      else if ((old_style_version == 10101) || (old_style_version == 111))
        data.version = 65793; // 1.1.1
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

  std::string current_version()
  {
    std::stringstream ss;
    ss << CURRENT_FIRMWARE_MAYOR_VERSION << "." << CURRENT_FIRMWARE_MINOR_VERSION << ".x";

    return std::string(ss.str());
  }

  int check_mayor_version()
  {
    // Return a negative value if firmware's mayor version is older than that of the driver,
    // 0 if both are the same, and a positive value if firmware's mayor version is newer
    uint32_t flashed_version = ((data.version & 0x00FF0000) >> 16);
    return flashed_version - CURRENT_FIRMWARE_MAYOR_VERSION;
  }

  int check_minor_version()
  {
    // Return a negative value if firmware's minor version is older than that of the driver,
    // 0 if both are the same, and a positive value if firmware's minor version is newer
    uint32_t flashed_version = ((data.version & 0x0000FF00) >> 8);
    return flashed_version - CURRENT_FIRMWARE_MINOR_VERSION;
  }
};

} // namespace kobuki

#endif /* KOBUKI_FW_DATA_HPP__ */

