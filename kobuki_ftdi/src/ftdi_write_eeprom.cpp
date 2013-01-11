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
 * @file src/ftdi_write_eeprom.cpp
 *
 * @brief Reads an eeprom binary saved as a file and write this to the chip.
 * Use this to rewrite good eeprom binaries (or our saved eeproms/eeprom.original)
 * when you've mangled one.
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
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{
  const int vendor_id = 0x0403;
  const int product_id = 0x6001;
  int result;

  /*********************
   ** Parse Command Line
   **********************/
  ecl::CmdLine cmd_line("This is used to write 'kobuki' on an ftdi device.", ' ', "0.1");
  ecl::ValueArg<std::string> serial_arg(
      "s", "serial", "Identify the device via the old serial id (if there are multiple attached) ['unspecified'].", false,
      "unspecified", "string");
  ecl::ValueArg<std::string> filename_arg(
      "f", "filename", "Name of the eeprom binary. ['eeprom.new'].", false,
      "eeprom.new", "string");
  cmd_line.add(serial_arg);
  cmd_line.add(filename_arg);
  cmd_line.parse(argc, argv);
  bool using_serial_id = false;
  std::string serial_id;
  if (serial_arg.getValue() != "unspecified")
  {
    using_serial_id = true;
    serial_id = serial_arg.getValue();
  }
  std::string filename = filename_arg.getValue();

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
   ** Load Eeeprom Binary
   *******************************************/
  std::cout << "Load eeprom binary" << std::endl;
  unsigned char eeprom_binary[512];
  FILE *fp = fopen(filename.c_str(), "rb");
  if ( fp == NULL ) {
    std::cerr << "Error: could not read the eeprom binary file." << std::endl;
    return EXIT_FAILURE;
  }
  size_t n = fread(eeprom_binary,1,512,fp);
  if ( ( n != 512 ) && ( !feof(fp) ) ) {
    std::cerr << "Error: failed read from eeoprom binary file" << std::endl;
  } else {
    std::cout << "  Size: " << n << " bytes" << std::endl;
  }
  fclose (fp);

  /*********************
  ** Erasing eeprom
  **********************/
  std::cout << "Erasing eeprom" << std::endl;
  result = ftdi_erase_eeprom(&ftdi);
  if ( result == -1 ) {
    std::cerr << "Error: erase failed." << std::endl;
    return EXIT_FAILURE;
  } else if ( result == -2 ) {
    std::cerr << "Error: usb device unavailable." << std::endl;
    return EXIT_FAILURE;
  } else {
    std::cout << "  Ok" << std::endl;
  }

  /*********************
  ** Flashing Eeprom
  **********************/
  std::cout << "Flashing eeprom" << std::endl;
  result = ftdi_write_eeprom(&ftdi, eeprom_binary);
  if (result < 0)
  {
    std::cerr << "Error : write failed [" << ftdi_get_error_string(&ftdi) << "]" << std::endl;
    return EXIT_FAILURE;
  } else {
    std::cout << "  Ok" << std::endl;
  }

  /*********************
  ** Cleanup
  **********************/
  std::cout << "Done." << std::endl;

  return 0;
}

