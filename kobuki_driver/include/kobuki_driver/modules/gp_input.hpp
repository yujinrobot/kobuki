/**
 * @file /include/kobuki_driver/modules/gp_input.hpp
 *
 * Module for handling of gpio data commands.
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_GPIO_HPP__
#define KOBUKI_GPIO_HPP__

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
  struct Data {
    Data() : gp_adc(7) {}
    uint16_t gp_input;
    std::vector<uint16_t> gp_adc;
  } data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_inertia: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(Header::GpInput, byteStream);
    buildBytes(data.gp_input, byteStream);
    for (unsigned int i = 0; i < data.gp_adc.size(); ++i)
    {
      buildBytes(data.gp_adc[i], byteStream);
    }
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_inertia: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id;
    buildVariable(header_id, byteStream);
    buildVariable(data.gp_input, byteStream);
    for (unsigned int i = 0; i < data.gp_adc.size(); ++i)
    {
      buildVariable(data.gp_adc[i], byteStream);
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
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, gp_adc[2], gp_adc[1], gp_adc[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_GPIO_HPP__ */

