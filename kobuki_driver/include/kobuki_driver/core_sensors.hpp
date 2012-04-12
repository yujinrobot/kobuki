#ifndef KOBUKI_DEFAULT_DATA_HPP__
#define KOBUKI_DEFAULT_DATA_HPP__

#include <ecl/containers.hpp>
#include <kobuki_comms/CoreSensors.h>
#include "packet_handler/payload_base.hpp"

namespace kobuki
{

class DefaultData : public packet_handler::payloadBase
{
public:
  kobuki_comms::CoreSensors data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace kobuki

#endif /* KOBUKI_DEFAULT_DATA_HPP__ */
