/**
 * @file /include/kobuki_driver/modules/hardware.hpp
 *
 * Module for handling of hardware version request packet payloads.
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
    uint16_t version;
  } data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_hw: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(Header::Hardware, byteStream);
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

    unsigned char header_id;
    buildVariable(header_id, byteStream);
    buildVariable(data.version, byteStream);

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

