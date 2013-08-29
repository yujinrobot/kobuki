/**
 * @file src/reset_device.cpp
 *
 * @brief Reset the all FTDI devices connected to load it again as standard tty device. After the flashing serial number.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
 **/
#include <iostream>

#include "kobuki_ftdi/scanner.hpp"

int main(int argc, char** argv)
{
  int ret_val;
  FTDI_Scanner scanner;

  if ( scanner.scan() < 0 ) {
    std::cerr << "Failed to find devices to reset." << std::endl;
    return -1;
  }

  ret_val = scanner.reset();
  if ( ret_val < 0 && ret_val != -19 )
    std::cerr << "Failed to reeset devices." << std::endl;
  return ret_val;
}
