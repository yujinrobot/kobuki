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
 * RACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file include/kobuki_ftdi/writer.hpp
 * @brief Provides common modulde for wrting serial number and other data into the eeprom of ftdi chip.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cstring>
#include <ftdi.h>

/*****************************************************************************
 ** Definition
 *****************************************************************************/

//! A Writer class.
/*!
 A Writer class for FTDI devices, especially for FTDI FT232R chip.
 */
class FTDI_Writer {
public:
  /**
   * A constructor, that more elaborated..
   */
  FTDI_Writer() :
    log_level(0),
    serial_only(false),
    use_first_device(false),
    vendor_id(0x0403),
    product_id(0x6001)
  {;}

  /**
   * reset flags to default state.
   * @sa write(const std::string&), write(const std::string&, const std::string&)
   */
  void reset_flags() {
    serial_only = false;
    use_first_device = false;
    return;
  }

  /**
   * Write new serial name to ftdi device what first detected.
   * @param [in] new_id_ new serial number for ftdi device
   * @return 0 on success<br>negative number on failure
   * @sa write(const std::string&, const std::string&), write(const std::string&, const std::string&, const std::string&, const std::string&)
   **/
  int write(const std::string &new_id_) {
    serial_only = true;
    use_first_device = true;
    int ret_val = write("", new_id_, "", "");
    reset_flags();

    return ret_val;
  }

  /**
   * Write new serial name to ftdi device what have specified serial name.
   * @param [in] old_id_ serial number of target device
   * @param [in] new_id_ new serial number for ftdi device
   * @return 0 on success<br>negative number on failure
   * @sa write(const std::string&), write(const std::string&, const std::string&, const std::string&, const std::string&)
   **/
  int write(const std::string &old_id_, const std::string &new_id_)
  {
    serial_only = true;
    int ret_val = write(old_id_, new_id_, "", "");
    reset_flags();

    return ret_val;
  }

  /**
   * Write new serial name to ftdi device what have specified serial name, manufacturer and product name also.
   * @param [in] old_id_ serial number of target device
   * @param [in] new_id_ new serial number for ftdi device
   * @param [in] manufacturer_ new manufacture name for ftdi device
   * @param [in] product_ new product name for ftdi device
   * @return 0 on success<br>negative number on failure
   * @sa write(const std::string&), write(const std::string&, const std::string&)
   **/
  int write(const std::string &old_id_, const std::string &new_id_, const std::string &manufacturer_, const std::string &product_)
  {
//    int ret;
//    int i, no_devices;
    struct ftdi_context ftdic;
//    struct ftdi_device_list *devlist, *curdev;
/*
    char manufacturer[128], description[128];
    char serial[128];
*/

    /* Initialization */
    if (ftdi_init(&ftdic) < 0)
    {
      std::cerr << "ftdi_init failed" << std::endl;
      return -1;
    }

//    if ((no_devices = ftdi_usb_find_all(&ftdic, &devlist, vendor_id, product_id)) < 0)
//    {
//      std::cerr << "ftdi_usb_find_all failed: " << ftdi_get_error_string(&ftdic) << std::endl;
//      return -1;
//    }
//    std::cout << "Number of FTDI devices found: " << no_devices << std::endl;
//    std::cout << std::endl;
//
//
//    i = 0;
//    for (curdev = devlist; curdev != NULL; i++)
//    {
//      if( i > no_devices ) break; // Exceptional cases
//
//      std::cout << "Target Device #" << i << std::endl;
#if 0
      if ((ret = ftdi_usb_get_strings(&ftdic, curdev->dev, manufacturer, 128, description, 128, serial, 128)) < 0)
      {
        std::cerr << "ftdi_usb_get_strings failed: " << ftdi_get_error_string(&ftdic) << "." << std::endl;
        return -1;
        /*
        if( std::string(ftdi_get_error_string(&ftdic)).find("Operation not permitted")  == -1 ) {
          continue;
        }
        else
          return EXIT_FAILURE;
        */
      }
      std::cout << "  Manufacturer: " << manufacturer << std::endl;
      std::cout << "  Description : " << description << std::endl;
      std::cout << "  Serial Id   : " << serial << std::endl;
      std::cout << std::endl;

      if ((!use_first_device) && (serial != old_id_)) {
        std::cout << "It is not intedend target." << std::endl;
        std::cout << "target: " << old_id_ << ", current: " << serial << std::endl;
        curdev = curdev->next;
        continue;
      }
#endif
      // Open the ftdi device
      if (use_first_device) {
        if (ftdi_usb_open(&ftdic, vendor_id, product_id) < 0)
        {
          std::cerr << "Couldn't find/open a ftdi device." << std::endl;
          return -1;
        }
      } else {
        if (ftdi_usb_open_desc(&ftdic, vendor_id, product_id, NULL, old_id_.c_str()) < 0)
        {
          std::cerr << "Couldn't open the device with serial id string " << old_id_ << std::endl;
          return -1;
        }
      }

      // Open an Eeprom
      //std::cout << "Eeprom Binary" << std::endl;
      ftdi_eeprom eeprom;
      unsigned char eeprom_binary[512];
      int result = ftdi_read_eeprom(&ftdic, eeprom_binary);
      int size = FTDI_DEFAULT_EEPROM_SIZE;
      // this never works for me
      // int size = ftdi_read_eeprom_getsize(&ftdi, eeprom_binary, 512);
      if (size < 0)
      {
        std::cerr << "Error: Could not read the eeprom from the requested device." << std::endl;
        return -1;
      }
      std::cout << "Read eeprom binary [" << size << " bytes]." << std::endl;

      std::cout << "Decoding into eeprom structure." << std::endl;
      // put the binary into an eeprom structure
      if ( ftdi_eeprom_decode(&eeprom, eeprom_binary, size) != 0 ) {
        std::cerr << "Error: Could not write raw binary eeprom into the eeprom structure." << std::endl;
        return -1;//EXIT_FAILURE;
      }

      std::string new_manufacturer = manufacturer_;
      std::string new_product = product_;
      std::string new_id = new_id_;

      //Preparnig eeprom data to write
      eeprom.chip_type = TYPE_R;
      eeprom.size = size;
      free(eeprom.serial);
      eeprom.serial = (char*)malloc(new_id.size() + 1);
      std::strcpy(eeprom.serial, new_id.c_str());
      if (!serial_only) {
        free(eeprom.manufacturer);
        eeprom.manufacturer = (char*)malloc(new_manufacturer.size() + 1);
        std::strcpy(eeprom.manufacturer, new_manufacturer.c_str());
        free(eeprom.product);
        eeprom.product = (char*)malloc(new_product.size() + 1);
        std::strcpy(eeprom.product, new_product.c_str());
      }

      /* Build new eeprom in binary */
      std::cout << "Building new eeprom binary." << std::endl;
      int eeprom_binary_length = ftdi_eeprom_build(&eeprom, eeprom_binary);
      if (eeprom_binary_length == -1)
      {
        std::cerr << "Eeprom binary exceeded 128 bytes, reduce the size of your strings." << std::endl;
        return -1;
        //continue;
        //return EXIT_FAILURE;
      }
      else if (eeprom_binary_length == -2)
      {
        std::cerr << "Eeprom structure not valid." << std::endl;
        return -1; //continue;
        //return EXIT_FAILURE;
      }

      /* Flashing eeprom on ftdi chip */
      std::cout << "Flashing eeprom binary." << std::endl;
      result = ftdi_write_eeprom(&ftdic, eeprom_binary);
      if (result < 0)
      {
        std::cerr << "  Could not rewrite the eeprom." << std::endl;
        return -1; //continue;
        //return EXIT_FAILURE;
      }
      std::cout << "  Flashed " << ftdic.eeprom_size << " bytes" << std::endl;

#if 0
      // Verification
      if ((ret = ftdi_usb_get_strings(&ftdic, curdev->dev, manufacturer, 128, description, 128, serial, 128)) < 0)
      {
        std::cerr << "ftdi_usb_get_strings failed: " << ftdi_get_error_string(&ftdic) << "." << std::endl;
        return -1;
      }
      std::cout << "Verify Device #" << i << std::endl;
      std::cout << "  Manufacturer: " << manufacturer << std::endl;
      std::cout << "  Description : " << description << std::endl;
      std::cout << "  Serial Id   : " << serial << std::endl;
      std::cout << std::endl;
#endif
      std::cout << "Done." << std::endl;
//
//      curdev = curdev->next;
//    }
//
//    ftdi_list_free(&devlist);
    ftdi_deinit(&ftdic);
    return 0;
  }

private:
  int log_level; /**< currently unused. */
  bool serial_only; /**< flags to control internal behavior of writer class. */
  bool use_first_device; /**< flags to control internal behavior of writer class. */

  /**
   * vendor ID of FTDI chip. Fixed to 0x0403 for FTDI company.
   */
  const unsigned short vendor_id;

  /**
   * product ID of FTDI chip. Fixed to 0x6001 for FT232R chip.
   */
  const unsigned short product_id;
};
