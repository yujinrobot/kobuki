/**                                                                           !
  * @file /kobuki_driver/src/test/simple_loop.cpp
  *
  * @brief Example/test program with simple loop.
  *
  * It provides simple example of how interact with kobuki by using c++ without ROS.
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include <csignal>
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
    parameters.enable_acceleration_limiter = false;
    // configure other parameters here
    kobuki.init(parameters);
    kobuki.enable();
    slot_stream_data.connect("/mobile_base/stream_data");
  }
  ~KobukiManager() {
    kobuki.setBaseControl(0,0);
    kobuki.disable();
  }

  void processStreamData() {
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    pose *= pose_update;
    dx += pose_update.x();
    dth += pose_update.heading();
    //std::cout << dx << ", " << dth << std::endl;
    //std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
    std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    processMotion();
  }

  void processMotion() {
    if (dx >= 1.0 && dth >= ecl::pi/2.0) { dx=0.0; dth=0.0; kobuki.setBaseControl(0.0, 0.0); return; }
    else if (dx >= 1.0) { kobuki.setBaseControl(0.0, 3.3); return; }
    else { kobuki.setBaseControl(0.3, 0.0); return; }
  }

private:
  double dx, dth;
  ecl::Pose2D<double> pose;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
};

/*****************************************************************************
** Signal Handler
*****************************************************************************/

volatile bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);

  std::cout << "life is not a simple loop." << std::endl;
  KobukiManager kobuki_manager;

  while (!shutdown_req);
  return 0;
}
