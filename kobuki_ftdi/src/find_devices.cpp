/**
 * @file src/find_devices.cpp
 *
 * @brief Detect the presence of FTDI devices.
 *
 * If any FTDI device is detected this program will exit with 0.
 * Or not, exit with -1.
 *
 * find_devices does not need to run with root privileges.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
 **/
#include <iostream>

#include "kobuki_ftdi/scanner.hpp"

int main(int argc, char** argv)
{
  FTDI_Scanner scanner;
  int no_devices = scanner.scan();
  if (no_devices < 0) {
    std::cout << "not found!!!" << std::endl;
    return -1;
  }
  std::cout << no_devices << " device(s) found." << std::endl;
  return 0;
}
