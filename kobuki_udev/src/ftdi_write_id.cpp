/**
 * @file /ftdi/src/utils/ftdi_write_id.cpp
 *
 * @brief Write an id string to the requested ftdi device.
 *
 * @date August 2010
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ftdi.h>
#include <cstring>
#include <ecl/command_line.hpp>

/*****************************************************************************
 ** Using
 *****************************************************************************/

using ecl::CmdLine;
using ecl::UnlabeledValueArg;
using ecl::ValueArg;
using ecl::SwitchArg;
using std::string;

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  const unsigned short vendor_id = 0x0403;
  const unsigned short product_id = 0x6001;

  /*********************
   ** Parse Command Line
   **********************/
  CmdLine cmd_line("This is used to write a new serial id string to the ftdi device.", ' ', "0.1");
  ValueArg<std::string> old_arg(
      "o", "old", "Identify the device via the old serial id (if there are multiple attached) ['unspecified'].", false,
      "unspecified", "string");
  UnlabeledValueArg<std::string> new_arg("new_id", "New serial id used to identify the device [new_id].", true,
                                         "new_id", "string");
  cmd_line.add(old_arg);
  cmd_line.add(new_arg);
  cmd_line.parse(argc, argv);
  bool using_old_id = false;
  string old_id;
  if (old_arg.getValue() != "unspecified")
  {
    using_old_id = true;
    old_id = old_arg.getValue();
  }
  string new_id = new_arg.getValue();

  /*********************
   ** Debug output
   **********************/
  std::cout << "Input Information" << std::endl;
  if (!using_old_id)
  {
    std::cout << "  Old id: unused (first device found.)" << std::endl;
  }
  else
  {
    std::cout << "  Old id: " << old_id << std::endl;
  }
  std::cout << "  New id: " << new_id << std::endl;

  /*********************
   ** Open a context
   **********************/
  struct ftdi_context ftdi;
  if (ftdi_init(&ftdi) < 0)
  {
    std::cerr << "ftdi_init failed" << std::endl;
    return EXIT_FAILURE;
  }
  if (!using_old_id)
  { // simply open up the first found.
    if (ftdi_usb_open(&ftdi, vendor_id, product_id) < 0)
    {
      std::cerr << "Couldn't find/open an ftdi device." << std::endl;
      return EXIT_FAILURE;
    }
  }
  else
  {
    if (ftdi_usb_open_desc(&ftdi, vendor_id, product_id, NULL, old_id.c_str()) < 0)
    {
      std::cerr << "Couldn't open the device with serial id string: " << old_id << std::endl;
      return EXIT_FAILURE;
    }
  }
  /*********************
   ** Open an Eeprom
   **********************/
  std::cout << "Eeprom" << std::endl;
  ftdi_eeprom eeprom;
  unsigned char eeprom_binary[512];
//	int result = ftdi_read_eeprom(&ftdi,eeprom_binary);
  int size = ftdi_read_eeprom_getsize(&ftdi, eeprom_binary, 512);
  if (size < 0)
  {
    std::cerr << "Could not read the eeprom from the requested device." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "  Read binary [" << size << " bytes]." << std::endl;
  std::cout << "  Decoding into eeprom structure." << std::endl;
  ftdi_eeprom_decode(&eeprom, eeprom_binary, ftdi.eeprom_size); // put the binary into an eeprom structure
  std::cout << "    Manufacturer: " << eeprom.manufacturer << std::endl;
  std::cout << "    Product     : " << eeprom.product << std::endl;
  std::cout << "    Vendor Id   : " << eeprom.vendor_id << std::endl;
  std::cout << "    Product Id  : " << eeprom.product_id << std::endl;
  std::cout << "    Serial Id   : " << eeprom.serial << std::endl;
  std::cout << "    Self Powered: " << eeprom.self_powered << std::endl;
  std::cout << "    Remote Wake : " << eeprom.remote_wakeup << std::endl;
  std::cout << "    In Iso      : " << eeprom.in_is_isochronous << std::endl;
  std::cout << "    Out Iso     : " << eeprom.out_is_isochronous << std::endl;
  std::cout << "    Suspend     : " << eeprom.suspend_pull_downs << std::endl;
  std::cout << "    Max Power   : " << eeprom.max_power << std::endl;
  // have to decode this one manually from the function above since it is not
  // actually stored in the eeprom binary
  eeprom.size = size;
  std::cout << "    Size        : " << eeprom.size << std::endl;
  std::cout << "  New serial id: " << new_id << "." << std::endl;
  free(eeprom.serial);
  eeprom.serial = (char*)malloc(new_id.size() + 1);
  std::strcpy(eeprom.serial, new_id.c_str());

  std::cout << "  Building new binary." << std::endl;
  int eeprom_binary_length = ftdi_eeprom_build(&eeprom, eeprom_binary);
  if (eeprom_binary_length == -1)
  {
    std::cerr << "Eeprom binary exceeded 128 bytes, reduce the size of your strings." << std::endl;
    return EXIT_FAILURE;
  }
  else if (eeprom_binary_length == -2)
  {
    std::cerr << "Eeprom structure not valid." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "  Writing binary to flash." << std::endl;
  int result = ftdi_write_eeprom(&ftdi, eeprom_binary);
  if (result < 0)
  {
    std::cerr << "Could not rewrite the eeprom." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done." << std::endl;

  return 0;
}

