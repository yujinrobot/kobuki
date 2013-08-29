/**
 * @file kobuki_driver/packets/dock_ir.hpp
 *
 * @brief Docking infrared sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_DOCK_IR_DATA_HPP__
#define KOBUKI_DOCK_IR_DATA_HPP__

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

class DockIR : public packet_handler::payloadBase
{
public:
  DockIR() : packet_handler::payloadBase(false, 3) {};
  struct Data {
    Data() : docking(3) {}
    std::vector<uint8_t> docking;
  } data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    buildBytes(Header::DockInfraRed, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.docking[0], byteStream);
    buildBytes(data.docking[1], byteStream);
    buildBytes(data.docking[2], byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (byteStream.size() < length+2)
    {
      //std::cout << "kobuki_node: kobuki_dock_ir: deserialise failed. not enough byte stream." << std::endl;
      return false;
    }

    unsigned char header_id, length_packed;
    buildVariable(header_id, byteStream);
    buildVariable(length_packed, byteStream);
    if( header_id != Header::DockInfraRed ) return false;
    if( length_packed != length ) return false;

    buildVariable(data.docking[0], byteStream);
    buildVariable(data.docking[1], byteStream);
    buildVariable(data.docking[2], byteStream);

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, docking[2], docking[1], docking[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_IR_DATA_HPP__ */
