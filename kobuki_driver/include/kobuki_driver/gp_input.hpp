#ifndef KOBUKI_GP_INPUT_DATA_HPP__
#define KOBUKI_GP_INPUT_DATA_HPP__

#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"
#include <kobuki_comms/GpInput.h>

namespace kobuki
{

class GpInputData : public packet_handler::payloadBase
{
public:
  // container
  kobuki_comms::GpInput data;

  GpInputData()
  {
    data.gp_adc.resize(7);
  }

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_inertia: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(data.header_id, byteStream);
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
      ROS_WARN_STREAM("kobuki_node: kobuki_inertia: deserialise failed. empty byte stream.");
      return false;
    }

    buildVariable(data.header_id, byteStream);
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

#endif /* KOBUKI_GP_INPUT_DATA_HPP__ */

