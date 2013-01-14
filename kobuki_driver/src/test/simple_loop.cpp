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
#include <ecl/geometry/pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#include "kobuki_driver/kobuki.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager() :
    dx(0.0), dth(0.0),
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
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    dx += pose_update.x();
    dth += pose_update.heading();

    processMotion();
  }

  void processMotion() {
    if (dx > 1.0 && dth > 3.141592) { dx=0.0; dth=0.0; kobuki.setBaseControl(0.3, 0.0); return; }
    else if (dx > 1.0) { kobuki.setBaseControl(0.0, 0.3); return; }
    else { kobuki.setBaseControl(0.3, 0.0); return; }
  }

private:
  double dx, dth;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
};


/*****************************************************************************
** Main
*****************************************************************************/

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
