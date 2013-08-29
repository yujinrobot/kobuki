/**
 * @file include/kobuki_driver/packets/gp_input.hpp
 *
 * @brief gpio data command packets.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_GP_INPUT_HPP__
#define KOBUKI_GP_INPUT_HPP__

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

class GpInput : public packet_handler::payloadBase
{
public:
  GpInput() : packet_handler::payloadBase(false, 16) {};
  struct Data {
    Data() : analog_input(4) {}
    uint16_t digital_input;
    /**
     * This currently returns 4 unsigned shorts containing analog values that
     * vary between 0 and 4095. These represent the values coming in on the
     * analog pins.
     */
    std::vector<uint16_t> analog_input;
  } data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    buildBytes(Header::GpInput, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.digital_input, byteStream);
    for (unsigned int i = 0; i < data.analog_input.size(); ++i)
    {
      buildBytes(data.analog_input[i], byteStream);
    }
    for (unsigned int i = 0; i < 3; ++i)
    {
      buildBytes(0x0000, byteStream); //dummy
    }
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (byteStream.size() < length+2)
    {
      //std::cout << "kobuki_node: kobuki_inertia: deserialise failed. not enough byte stream." << std::endl;
      return false;
    }

    unsigned char header_id, length_packed;
    buildVariable(header_id, byteStream);
    buildVariable(length_packed, byteStream);
    if( header_id != Header::GpInput ) return false;
    if( length_packed != length ) return false;

    buildVariable(data.digital_input, byteStream);

    //for (unsigned int i = 0; i < data.analog_input.size(); ++i)
    // It's actually sending seven 16-bit variables.
    // 0-3 : the analog pin inputs
    // 4 : ???
    // 5-6 : 0
    for (unsigned int i = 0; i < 4; ++i)
    {
      buildVariable(data.analog_input[i], byteStream);
    }
    for (unsigned int i = 0; i < 3; ++i) {
      uint16_t dummy;
      buildVariable(dummy, byteStream);
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

#endif /* KOBUKI_GP_INPUT_HPP__ */

