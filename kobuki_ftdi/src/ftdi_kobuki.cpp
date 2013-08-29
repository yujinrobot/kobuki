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
 * @file src/ftdi_kobuki.cpp
 *
 * @brief Write an id string to the requested ftdi device.
 *
 * This is actually acutely prepared for setting up kobuki.
 * Note that it makes an eeprom.backup binary for you just in case things
 * go wrong - when this happens, simply call:
 *
 * @code
 * $ sudo ./bin/ftdi_write_eeprom -f eeprom.backup
 * @endcode
 * to restore.
 *
 * What does this program actually do?
 *
 * - reads an eeprom binary from the device
 * - saves it as eeprom.backup
 * - decodes it into an eeprom structure
 * - converts the serial, manufacture and product strings
 * - builds the eeprom binary from this
 * - flashes the eeprom binary to the device
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
  UnlabeledValueArg<std::string> new_arg("new_id", "New serial id used to identify the device [kobuki].", false,
                                         "kobuki", "string");
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
  std::string new_manufacturer("Yujin Robot");
  std::string new_product("iClebo Kobuki");

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
  std::cout << "Eeprom Binary" << std::endl;
  ftdi_eeprom eeprom;
  unsigned char eeprom_binary[512];
  int result = ftdi_read_eeprom(&ftdi,eeprom_binary);
  int size = FTDI_DEFAULT_EEPROM_SIZE;
  // this never works for me
//  int size = ftdi_read_eeprom_getsize(&ftdi, eeprom_binary, 512);
  if (size < 0)
  {
    std::cerr << "Error: Could not read the eeprom from the requested device." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "  Read binary [" << size << " bytes]." << std::endl;
  std::cout << "  Saving binary ['eeprom.backup']" << std::endl;
  FILE *fp = fopen ("eeprom.backup", "wb");
  fwrite (&eeprom_binary, 1, size, fp);
  fclose (fp);

  std::cout << "Decoding into eeprom structure." << std::endl;
  // put the binary into an eeprom structure
  if ( ftdi_eeprom_decode(&eeprom, eeprom_binary, size) != 0 ) {
    std::cerr << "Error: Could not write raw binary eeprom into the eeprom structure." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "    Vendor Id   : " << eeprom.vendor_id << std::endl;
  std::cout << "    Product Id  : " << eeprom.product_id << std::endl;
  std::cout << "    Self Powered: " << eeprom.self_powered << std::endl;
  std::cout << "    Remote Wake : " << eeprom.remote_wakeup << std::endl;
  std::cout << "    In Iso      : " << eeprom.in_is_isochronous << std::endl;
  std::cout << "    Out Iso     : " << eeprom.out_is_isochronous << std::endl;
  std::cout << "    Suspend     : " << eeprom.suspend_pull_downs << std::endl;
  std::cout << "    Use Serial  : " << eeprom.use_serial << std::endl;
  std::cout << "    Change Usb  : " << eeprom.change_usb_version << std::endl;
  std::cout << "    Usb Version : " << eeprom.usb_version << std::endl;
  std::cout << "    Max Power   : " << eeprom.max_power << std::endl;
  std::cout << "    Manufacturer: " << eeprom.manufacturer << std::endl;
  std::cout << "    Product     : " << eeprom.product << std::endl;
  std::cout << "    Serial Id   : " << eeprom.serial << std::endl;
  if ( eeprom.chip_type == TYPE_R ) {
    std::cout << "    Chip Type   : TYPE_R" << std::endl;
  } else {
    std::cout << "    Chip Type   : " << eeprom.chip_type << std::endl;
  }
  // Looks like the eeprom structure gets TYPE_AM (0)...if we reprogram with
  // that, for some reason the chip fails. Guesses: maybe this variable is
  // like the size variable (below), it isn't stored in the eeprom and
  // we have to manually set it before flashing.
  eeprom.chip_type = TYPE_R;

//  std::cout << "    Invert      : " << eeprom.invert << std::endl;

  new_id = new_id + "_" + std::string(eeprom.serial);

  // The size is not stored in the eeprom on the chip itself...rather this
  // variable is used when passing eeprom_binary to the ftdi_eeprom_build command.
  eeprom.size = size;
  std::cout << "    Size        : " << eeprom.size << std::endl;
  std::cout << "  New serial    : " << new_id << "." << std::endl;
  std::cout << "  New manufact. : " << new_manufacturer << "." << std::endl;
  std::cout << "  New product   : " << new_product << "." << std::endl;
  std::cout << "  New chip type : TYPE_R." << std::endl;
  free(eeprom.serial);
  eeprom.serial = (char*)malloc(new_id.size() + 1);
  std::strcpy(eeprom.serial, new_id.c_str());
  free(eeprom.manufacturer);
  eeprom.manufacturer = (char*)malloc(new_manufacturer.size() + 1);
  std::strcpy(eeprom.manufacturer, new_manufacturer.c_str());
  free(eeprom.product);
  eeprom.product = (char*)malloc(new_product.size() + 1);
  std::strcpy(eeprom.product, new_product.c_str());

  std::cout << "Building new eeprom binary." << std::endl;
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
//  std::cout << "  Saving binary ['eeprom.req.after']" << std::endl;
//  fp = fopen ("eeprom.req.after", "wb");
//  fwrite (&eeprom_binary, 1, size, fp);
//  fclose (fp);

  std::cout << "  Flashing binary." << std::endl;
  result = ftdi_write_eeprom(&ftdi, eeprom_binary);
  if (result < 0)
  {
    std::cerr << "  Could not rewrite the eeprom." << std::endl;
    return EXIT_FAILURE;
  } else {
    std::cout << "  Flashed " << result << " bytes" << std::endl;
  }
  std::cout << "Done." << std::endl;

  return 0;
}

