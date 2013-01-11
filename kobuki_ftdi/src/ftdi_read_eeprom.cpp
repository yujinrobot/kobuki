/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file src/ftdi_read_eeprom.cpp
 *
 * @brief Reads a chip and saves the result as an eeprom binary file.
 *
 * Note that I can't get the requested size one to actually work.
 * So I've commented that out for now.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ftdi.h>
#include <cstring>
#include <ecl/command_line.hpp>

/*****************************************************************************
** Preprocessor
*****************************************************************************/

#define REQUEST_EEPROM_SIZE 0

/*****************************************************************************
 ** Using
 *****************************************************************************/

using ecl::CmdLine;
using ecl::UnlabeledValueArg;
using ecl::ValueArg;
using ecl::SwitchArg;
using std::string;

/*****************************************************************************
** Functions
*****************************************************************************/

bool decode(unsigned char* eeprom_binary, const int &size) {
  ftdi_eeprom eeprom;
  std::cout << "  Decoding into eeprom structure." << std::endl;
  // put the binary into an eeprom structure
  if ( ftdi_eeprom_decode(&eeprom, eeprom_binary, size) != 0 ) {
    return false;
  }
  std::cout << "Eeprom:" << std::endl;
  std::cout << "    Manufacturer: " << eeprom.manufacturer << std::endl;
  std::cout << "    Product     : " << eeprom.product << std::endl;
  std::cout << "    Vendor Id   : " << eeprom.vendor_id << std::endl;
  std::cout << "    Product Id  : " << eeprom.product_id << std::endl;
  std::cout << "    Serial Id   : " << eeprom.serial << std::endl;
  std::cout << "    Self Powered: " << eeprom.self_powered << std::endl;
  std::cout << "    Remote Wake : " << eeprom.remote_wakeup << std::endl;
  std::cout << "    Use Serial  : " << eeprom.use_serial << std::endl;
  std::cout << "    In Iso      : " << eeprom.in_is_isochronous << std::endl;
  std::cout << "    Out Iso     : " << eeprom.out_is_isochronous << std::endl;
  std::cout << "    Suspend     : " << eeprom.suspend_pull_downs << std::endl;
  std::cout << "    Max Power   : " << eeprom.max_power << std::endl;
  std::cout << "    Chip Type   : " << eeprom.chip_type << std::endl;
  return true;
}
/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  const int vendor_id = 0x0403;
  const int product_id = 0x6001;

  /*********************
   ** Parse Command Line
   **********************/
  CmdLine cmd_line("This is used to read an eeprom on an ftdi device.", ' ', "0.1");
  ValueArg<std::string> serial_arg(
      "s", "serial", "Identify the device via the old serial id (if there are multiple attached) ['unspecified'].", false,
      "unspecified", "string");
  cmd_line.add(serial_arg);
  cmd_line.parse(argc, argv);
  bool using_serial_id = false;
  string serial_id;
  if (serial_arg.getValue() != "unspecified")
  {
    using_serial_id = true;
    serial_id = serial_arg.getValue();
  }

  /*********************
   ** Debug output
   **********************/
  std::cout << "Input Information" << std::endl;
  if (!using_serial_id)
  {
    std::cout << "  Serial id: unused (first device found.)" << std::endl;
  }
  else
  {
    std::cout << "  Serial id: " << serial_id << std::endl;
  }

  /*********************
   ** Open a context
   **********************/
  struct ftdi_context ftdi;
  if (ftdi_init(&ftdi) < 0)
  {
    std::cerr << "ftdi_init failed" << std::endl;
    return EXIT_FAILURE;
  }
  if (!using_serial_id)
  { // simply open up the first found.
    if (ftdi_usb_open(&ftdi, vendor_id, product_id) != 0)
    {
      std::cerr << "Couldn't find/open an ftdi device [" << ftdi.error_str << "]" << std::endl;
      return EXIT_FAILURE;
    }
  }
  else
  {
    if (ftdi_usb_open_desc(&ftdi, vendor_id, product_id, NULL, serial_id.c_str()) < 0)
    {
      std::cerr << "Couldn't open the device with serial id string: " << serial_id << std::endl;
      return EXIT_FAILURE;
    }
  }
  /******************************************
   ** Eeeprom Binary (Requested size)
   *******************************************/
  std::cout << "Eeprom Binary" << std::endl;
  unsigned char eeprom_binary[512];
  int size = ftdi_read_eeprom_getsize(&ftdi, eeprom_binary, 512);
  if (size < 0)
  {
    std::cerr << "Error: Could not read the eeprom from the requested device." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "  Read binary [" << size << " bytes]." << std::endl;
  std::cout << "  Write binary [eeprom.req]." << std::endl;
  FILE *fp = fopen ("eeprom.req", "wb");
  fwrite (&eeprom_binary, 1, size, fp);
  fclose (fp);

  if ( !decode(eeprom_binary, size) ) {
    std::cerr << "Error: Could not write raw binary eeprom into the eeprom structure." << std::endl;
    return EXIT_FAILURE;
  }

#if REQUEST_EEPROM_SIZE
  /******************************************
   ** Eeeprom Binary (Fixed size)
   *******************************************/
  // this is what the official example does.
  size = 128;
  int result = ftdi_read_eeprom(&ftdi,eeprom_binary);
  if ( result < 0 ) {
    std::cerr << "Error: Could not read the eeprom from the requested device." << std::endl;
    return EXIT_FAILURE;
  }
  fp = fopen ("eeprom.fix", "wb");
  fwrite (&eeprom_binary, 1, size, fp);
  fclose (fp);
  std::cout << "  Write binary [eeprom.fix]." << std::endl;

  if ( !decode(eeprom_binary, size) ) {
    std::cerr << "Error: Could not write raw binary eeprom into the eeprom structure." << std::endl;
    return EXIT_FAILURE;
  }
#endif
  std::cout << "Done." << std::endl;

  return 0;
}

