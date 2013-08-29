/**
 * @file /kobuki_driver/src/test/initialisation.cpp
 *
 * @brief Demo program for kobuki initialisation.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <kobuki_driver/kobuki.hpp>
#include <ecl/time.hpp>

class KobukiManager {
public:
  KobukiManager() {
    kobuki::Parameters parameters;
    // change the default device port from /dev/kobuki to /dev/ttyUSB0
    parameters.device_port = "/dev/ttyUSB0";
    // Other parameters are typically happy enough as defaults
    // namespaces all sigslot connection names under this value, only important if you want to
    parameters.sigslots_namespace = "/kobuki";
    // Most people will prefer to do their own velocity smoothing/acceleration limiting.
    // If you wish to utilise kobuki's minimal acceleration limiter, set to true
    parameters.enable_acceleration_limiter = false;
    // If your battery levels are showing significant variance from factory defaults, adjust thresholds.
    // This will affect the led on the front of the robot as well as when signals are emitted by the driver.
    parameters.battery_capacity = 16.5;
    parameters.battery_low = 14.0;
    parameters.battery_dangerous = 13.2;

    // initialise - it will throw an exception if parameter validation or initialisation fails.
    try {
      kobuki.init(parameters);
    } catch ( ecl::StandardException &e ) {
      std::cout << e.what();
    }
  }
private:
  kobuki::Kobuki kobuki;
};

int main() {
  KobukiManager kobuki_manager;
  ecl::Sleep()(5);
  return 0;
}
