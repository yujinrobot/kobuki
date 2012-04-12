#ifndef KOBUKI_CURRENT_DATA_HPP__
#define KOBUKI_CURRENT_DATA_HPP__

#include <vector>
#include <ecl/containers.hpp>
#include "../packet_handler/payload_base.hpp"

namespace kobuki
{

class CurrentData : public packet_handler::payloadBase
{
public:
  // container
  struct Data {
    Data() : current(2) {}
    uint8_t header_id;
    std::vector<uint8_t> current;
  } data;

  CurrentData()
  {
  }

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_current: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(data.header_id, byteStream);
    buildBytes(data.current[0], byteStream);
    buildBytes(data.current[1], byteStream);
    //buildBytes( data.current[2], byteStream );
    //buildBytes( data.current[3], byteStream );
    //buildBytes( data.current[4], byteStream );
    //buildBytes( data.current[5], byteStream );
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_current: deserialise failed. empty byte stream.");
      return false;
    }

    buildVariable(data.header_id, byteStream);
    buildVariable(data.current[0], byteStream);
    buildVariable(data.current[1], byteStream);
    //buildVariable( data.current[2], byteStream );
    //buildVariable( data.current[3], byteStream );
    //buildVariable( data.current[4], byteStream );
    //buildVariable( data.current[5], byteStream );

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, current[2], current[1], current[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_CURRENT_DATA_HPP__ */
