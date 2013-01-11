/**
 * @file src/get_serial_number.cpp
 *
 * @brief Print serial number of all the FTDI devices connected. With name of manufacture and product.
 *
 * It will not hurt any current connection.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
*/
#include <iostream>

#include "kobuki_ftdi/scanner.hpp"

int main(int argc, char** argv)
{
  int ret_val;
  FTDI_Scanner scanner;

  ret_val = scanner.scan();
  if (ret_val <= 0) {
    std::cout << "FTDI device not found!!!" << std::endl;
    return -1;
  }

  unsigned int no_devices = (unsigned int)ret_val;
  std::cout << no_devices << " device(s) found." << std::endl;

  std::string serial_number, manufacturer, product;
  for( unsigned int i=0; i<no_devices; i++ )
  {
    ret_val = scanner.get_serial_id(i, serial_number);
    if (ret_val < 0) break;

    ret_val = scanner.get_manufacturer(i, manufacturer);
    if (ret_val < 0) break;

    ret_val = scanner.get_product(i, product);
    if (ret_val < 0) break;

    std::cout << std::endl;
    std::cout << "Device #" << i << std::endl;
    std::cout << "  Manufacturer : " << manufacturer << std::endl;
    std::cout << "  Product      : " << product << std::endl;
    std::cout << "  Serial Number: " << serial_number << std::endl;
    ret_val = 0;
  }

  if (ret_val < 0) std::cerr << "Something went wrong. Did you run with sudo." << std::endl;
  return ret_val;
}
