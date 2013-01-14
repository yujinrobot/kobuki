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

class KobukiManager {
public:
  KobukiManager() :
    x(0.0), y(0.0), th(0.0),
    dx(0.0), dth(0.0),
    vx(0.0), wz(0.0),
    slot_stream_data(&KobukiManager::processStreamData, *this) // establish the callback
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/mobile_base"; // configure the first part of the sigslot namespace
    parameters.device_port = "/dev/kobuki";         // the serial port to connect to (windows COM1..)
    // configure other parameters here
    kobuki.init(parameters);
    slot_stream_data.connect("/mobile_base/stream_data");
  }

  void processStreamData() {
    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    std::cout << "Encoders [" <<  data.left_encoder << "," << data.right_encoder << "]" << std::endl;

    updateOdometry(data.left_encoder, data.right_encoder);
    processMotion();
    commandMotion();
  }

  void updateOdometry(double left, double right) {
    ;
  }

  void processMotion() {
    if (dx > 1.0 && dth > 3.141592) { dx=0.0; dth=0.0; vx=0.3, wz=0.0; return; }
    else if (dx >1.0) { vx=0.0, wz=0.3; return; }
    else { vx=0.3, wz=0.0; return; }
  }

  void commandMotion() {
    kobuki.setBaseControl(vx, wz);
  }

private:
  double x, y, th;
  double dx, dth;
  double vx, wz;

  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
};


int main(int arcc, char** argv)
{
  std::cout << "life is not a simple loop." << std::endl;

  bool shutdown_req = false;
  ecl::Sleep sleep(1);
  while (!shutdown_req){
    sleep();
    std::cout << "l" << std::endl;
    // do motion control
  }
  return 0;
}
