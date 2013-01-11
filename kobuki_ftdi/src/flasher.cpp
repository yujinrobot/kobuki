/**
 * @file src/flasher.cpp
 *
 * @brief Flash ftdi chip's serial number with 'kobuki_' prefix.
 *
 * Flash automatically when there is not flashed FTDI devices.
 * (Later)This program accept loop mode
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
 **/

#include <iostream>

#include "../include/kobuki_ftdi/scanner.hpp"
#include "../include/kobuki_ftdi/writer.hpp"

int main(int argc, char** argv)
{
  int ret_val;
  FTDI_Scanner scanner;
  FTDI_Writer writer;


  /*-------- 1 --------
         SCANNING
    -------- 1 --------*/
  // check and find ftdi device
  ret_val = scanner.scan();
  if( ret_val < 0 )
  {
    std::cerr << "not found!!!" << std::endl;
    return -1; // if not found
  }
  unsigned int no_devices = (unsigned)ret_val;


  /*-------- 2 --------
          WRITING
    -------- 2 --------*/
  for (unsigned int i=0; i<no_devices; i++) {
    // get serial id
    std::string serial_id;
    ret_val = scanner.get_serial_id(i, serial_id);
    if (ret_val == -2) {
      std::cout << ret_val << ": ";
      std::cout << "failed to get serial_id. did you run with sudo?" << std::endl;
      return -2; // if failed to get serial id
    }
    if (ret_val == -1) {
      std::cout << ret_val << ": ";
      std::cout << "Something went wrong." << std::endl;
      return -1;
    }

    //writing kobuki as serial id
    if( serial_id.substr(0,6) == std::string("kobuki") )
    {
      std::cout << "Already flashed as [" << serial_id << "]." << std::endl;
      //return 1; // if already kobuki
      continue;
    }

    std::string new_id = "kobuki_" + serial_id; // should be less than 20 character.
    // find the device that serial nummber is 'serial_id', and replace it with 'new_id.substr(0,20)'
    ret_val = writer.write( serial_id, new_id.substr(0,20), "Yujin Robot", "iClebo Kobuki" );
    if (ret_val < 0) {
      std::cout << ret_val << ": ";
      std::cout << "Something went wrong." << std::endl;
      return -1; // if failed to writing kobuki
    }
  }


  /*-------- 3 --------
         RESETTING
    -------- 3 --------*/
  //std::cout << "done." << std::endl;
  ret_val = scanner.reset();
  if (ret_val < 0 && ret_val != -19) {
    std::cout << ret_val << ": ";
    std::cout << "Something went wrong." << std::endl;
    return -1; // if failed to reset ftdi chip
  }
  return 0; // if success to reset.
}
