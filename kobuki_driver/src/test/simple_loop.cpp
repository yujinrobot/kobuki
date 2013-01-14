/**                                                                           !
  * @file /kobuki_driver/src/test/simple_loop.cpp
  *
  * @brief Example/test program with simple loop.
  *
  * It provides simple example of how interact with kobuki.
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include "kobuki_driver/kobuki.hpp"

kobuki::Kobuki robot;

void processStreamData() {
  kobuki::CoreSensors::Data data = robot.getCoreSensorData();
  std::cout << "Encoders [" <<  data.left_encoder << "," << data.right_encoder << "]" << std::endl;
}

int main(int arcc, char** argv)
{
  std::cout << "life is a simple loop." << std::endl;

  kobuki::Parameters parameters;

  parameters.sigslots_namespace = "/kobuki"; // configure the first part of the sigslot namespace
  parameters.device_port = "/dev/kobuki";    // the serial port to connect to (windows COM1..)
  robot.init(parameters);

  ecl::Slot<> slot_stream_data(processStreamData);
  slot_stream_data.connect("/kobuki/stream_data");

  bool shutdown_req = false;
  ecl::Sleep sleep(1);
  while (!shutdown_req){
    sleep();
    std::cout << "l" << std::endl;
  }
  return 0;
}
