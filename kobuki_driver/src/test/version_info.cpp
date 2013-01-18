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

#include "kobuki_driver/kobuki.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager() :
    acquired(false),
    slot_version_info(&KobukiManager::processVersionInfo, *this) // establish the callback
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/kobuki"; // configure the first part of the sigslot namespace
    parameters.device_port = "/dev/kobuki";    // the serial port to connect to (windows COM1..)
    kobuki.init(parameters);
    kobuki.enable();
    slot_version_info.connect("/kobuki/version_info");
  }

  ~KobukiManager() {
    kobuki.disable();
  }

  void processVersionInfo(const kobuki::VersionInfo &version_info) {
    hardware = kobuki::VersionInfo::toString(version_info.hardware);
    firmware = kobuki::VersionInfo::toString(version_info.firmware);
    software = kobuki::VersionInfo::toString(version_info.software);
    udid = kobuki::VersionInfo::toString(version_info.udid0, version_info.udid1, version_info.udid2);
    acquired = true;
  }

  bool isAcquired() { return acquired; }
  std::string& getHardwareVersion() { return hardware; }
  std::string& getFirmwareVersion() { return firmware; }
  std::string& getSoftwareVersion() { return software; }
  std::string& getUDID() { return udid; }

private:
  volatile bool acquired;
  kobuki::Kobuki kobuki;
  std::string hardware, firmware, software, udid;
  ecl::Slot<const kobuki::VersionInfo&> slot_version_info;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  std::cout << "Version Info:" << std::endl;
  KobukiManager kobuki_manager;

  while (!kobuki_manager.isAcquired());
  std::cout << " * Hardware Version: " << kobuki_manager.getHardwareVersion() << std::endl;
  std::cout << " * Firmware Version: " << kobuki_manager.getFirmwareVersion() << std::endl;
  std::cout << " * Software Version: " << kobuki_manager.getSoftwareVersion() << std::endl;
  std::cout << " * Unique Device ID: " << kobuki_manager.getUDID() << std::endl;

  return 0;
}
