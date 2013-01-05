/**
 * @file /include/kobuki_driver/packets/cliff.hpp
 *
 * @brief Cliff sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_CLIFF_DATA_HPP__
#define KOBUKI_CLIFF_DATA_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include <vector>
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

class Cliff : public packet_handler::payloadBase
{
public:
  struct Data {
    Data() : bottom(3) {}
    std::vector<uint16_t> bottom;
  } data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_cliff: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 6;
    buildBytes(Header::Cliff, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.bottom[0], byteStream);
    buildBytes(data.bottom[1], byteStream);
    buildBytes(data.bottom[2], byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_cliff: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id, length;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);
    buildVariable(data.bottom[0], byteStream);
    buildVariable(data.bottom[1], byteStream);
    buildVariable(data.bottom[2], byteStream);

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, bottom[2], bottom[1], bottom[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_IR_DATA_HPP__ */
