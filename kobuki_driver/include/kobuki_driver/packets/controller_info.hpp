/**
 * @file kobuki_driver/packets/controller_info.hpp
 *
 * @brief Docking infrared sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_CONTROLLER_INFO_HPP__
#define KOBUKI_CONTROLLER_INFO_HPP__

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

class ControllerInfo : public packet_handler::payloadBase
{
public:
  ControllerInfo() : packet_handler::payloadBase(false, 13) {};
  struct Data {
    Data() : type(0), p_gain(100*1000), i_gain(100), d_gain(2*1000) {}
    unsigned char type;
    unsigned int p_gain; //default value: 100 * 1000
    unsigned int i_gain; //default value: 0.1 * 1000
    unsigned int d_gain; //default value:   2 * 1000
  } data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    buildBytes(Header::ControllerInfo, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.type, byteStream);
    buildBytes(data.p_gain, byteStream);
    buildBytes(data.i_gain, byteStream);
    buildBytes(data.d_gain, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (byteStream.size() < length+2)
    {
      //std::cout << "kobuki_node: kobuki_controller_info: deserialise failed. not enough byte stream." << std::endl;
      return false;
    }

    unsigned char header_id, length_packed;
    buildVariable(header_id, byteStream);
    buildVariable(length_packed, byteStream);
    if( header_id != Header::ControllerInfo ) return false;
    if( length_packed != length ) return false;

    buildVariable(data.type, byteStream);
    buildVariable(data.p_gain, byteStream);
    buildVariable(data.i_gain, byteStream);
    buildVariable(data.d_gain, byteStream);

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

#endif /* KOBUKI_IR_DATA_HPP__ */
