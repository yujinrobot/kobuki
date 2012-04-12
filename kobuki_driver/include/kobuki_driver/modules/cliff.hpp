#ifndef KOBUKI_CLIFF_DATA_HPP__
#define KOBUKI_CLIFF_DATA_HPP__

#include <vector>
#include <ecl/containers.hpp>
#include "../packet_handler/payload_base.hpp"


namespace kobuki
{

class CliffData : public packet_handler::payloadBase
{
public:
  // container
  struct Data {
    Data() : bottom(3) {}
    uint8_t header_id;
    std::vector<uint16_t> bottom;
  } data;

  CliffData()
  {
  }

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_cliff: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(data.header_id, byteStream);
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

    buildVariable(data.header_id, byteStream);
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
