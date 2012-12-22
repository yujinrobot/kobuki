#include <iostream>

#include "../include/kobuki_ftdi/scanner.hpp"
#include "../include/kobuki_ftdi/writer.hpp"

int main(int argc, char** argv)
{
  /*-------- 1 --------
         SCANNING
    -------- 1 --------*/
  // check ftdi device
  int ret_val;
  FTDI_Scanner scanner;
  FTDI_Writer writer;

  ret_val = scanner.scan();
  if( ret_val < 0 )
  {
    std::cout << "ret_val: " << ret_val << " - "  ;
    std::cout << "someting happen."; 
    std::cout  << std::endl;
    return -1; // if not found
  }

  // get serial id
  std::string serial_id;
  ret_val = scanner.get_serial_id(serial_id);
  if (ret_val == -2) { 
    std::cout << "ret_val: " << ret_val << " - " ;
    std::cout << "failed to get serial_id. did you run with sudo?"; 
    std::cout  << std::endl;
    return -2; // if failed to get serial id
  }
  if (ret_val == -1) {
    std::cout << "ret_val: " << ret_val << " - "  ;
    std::cout << "someting happen."; 
    std::cout  << std::endl;
    return -1;
  }


  /*-------- 2 --------
          WRITING
    -------- 2 --------*/
  //writing kobuki as serial id
  if( serial_id.substr(0,6) == std::string("kobuki") ) 
  {
    std::cout << "ret_val: " << ret_val << " - "  ;
    std::cout << "already flashed."; 
    std::cout  << std::endl;
    return 1; // if already kobuki
  }

  std::string new_id = "kobuki_" + serial_id;///should be less than 20 character.
  ret_val = writer.write( new_id.substr(0,20), serial_id );
  if (ret_val < 0)   {
    std::cout << "ret_val: " << ret_val << " - "  ;
    std::cout << "someting happen."; 
    std::cout  << std::endl;
    return -1; // if failed to writing kobuki
  }


  /*-------- 3 --------
         RESETTING
    -------- 3 --------*/
  std::cout << "done." << std::endl;
  ret_val = scanner.reset();
  if (ret_val < 0 && ret_val != -19)   {
    std::cout << "ret_val: " << ret_val << " - "  ;
    std::cout << "someting happen."; 
    std::cout  << std::endl;
    return -1; // if failed to writing kobuki
  }
  return 0; // if success to writing kobuki
}
