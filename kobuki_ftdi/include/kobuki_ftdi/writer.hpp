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
 * @file src/ftdi_scan.cpp
 *
 * @brief Scan existing ftdi chips and print out their serial id strings.
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <cstring>
#include <ftdi.h>

/*****************************************************************************
 ** Main
 *****************************************************************************/
class FTDI_Writer {
public:
  int write(std::string new_id_, std::string old_id_)
  {
    const unsigned short vendor_id = 0x0403;
    const unsigned short product_id = 0x6001;
  
    int ret, i, no_devices;
    struct ftdi_context ftdic;
    struct ftdi_device_list *devlist, *curdev;
    char manufacturer[128], description[128];
    char serial[128];
  
    if (ftdi_init(&ftdic) < 0)
    {
      std::cerr << "ftdi_init failed" << std::endl;
      return -1;
    }
  
    if ((no_devices = ftdi_usb_find_all(&ftdic, &devlist, 0x0403, 0x6001)) < 0)
    {
      std::cerr << "ftdi_usb_find_all failed: " << ftdi_get_error_string(&ftdic) << std::endl;
      return -1;
    }
  
    std::cout << "Number of FTDI devices found: " << no_devices << std::endl;
  
    i = 0;
    for (curdev = devlist; curdev != NULL; i++)
    {
      std::cout << "Device #" << i << std::endl;
      if( i > no_devices ) break;
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

      if (serial != old_id_) {
        std::cout << "It is not intedend target." << std::endl;
        continue; 
      }
  
      if (ftdi_usb_open(&ftdic, vendor_id, product_id) < 0)
      {
        std::cerr << "Couldn't find/open an ftdi device." << std::endl;
        return -1;
      }
      /*********************
       ** Open an Eeprom
       **********************/
      std::cout << "Eeprom Binary" << std::endl;
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
      std::cout << "  Read binary [" << size << " bytes]." << std::endl;
      
      //std::cout << "  Saving binary ['eeprom.backup']" << std::endl;
      //FILE *fp = fopen ("eeprom.backup", "wb");
      //fwrite (&eeprom_binary, 1, size, fp);
      //fclose (fp);
      
      std::cout << "Decoding into eeprom structure." << std::endl;
      // put the binary into an eeprom structure
      if ( ftdi_eeprom_decode(&eeprom, eeprom_binary, size) != 0 ) {
        std::cerr << "Error: Could not write raw binary eeprom into the eeprom structure." << std::endl;
        return -1;//EXIT_FAILURE;
      }
      
      std::string new_manufacturer("Yujin Robot");
      std::string new_product("iClebo Kobuki");
      std::string new_id = new_id_;
      
      //Preparnig eeprom data to write
      eeprom.chip_type = TYPE_R;
      eeprom.size = size;
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
        continue;
        //return EXIT_FAILURE;
      }
      else if (eeprom_binary_length == -2)
      {
        std::cerr << "Eeprom structure not valid." << std::endl;
        continue;
        //return EXIT_FAILURE;
      }
  
      std::cout << "  Flashing binary." << std::endl;
      result = ftdi_write_eeprom(&ftdic, eeprom_binary);
      if (result < 0)
      {
        std::cerr << "  Could not rewrite the eeprom." << std::endl;
        continue;
        //return EXIT_FAILURE;
      } else {
        std::cout << "  Flashed " << result << " bytes" << std::endl;
       // kobuki_msgs::Led led;
       // led.value = kobuki_msgs::Led::GREEN;
       // led_publisher.publish(led);

        // Verification
        if ((ret = ftdi_usb_get_strings(&ftdic, curdev->dev, manufacturer, 128, description, 128, serial, 128)) < 0)
        {
          std::cerr << "ftdi_usb_get_strings failed: " << ftdi_get_error_string(&ftdic) << "." << std::endl;
          return -1;
        }
        std::cout << "  Manufacturer: " << manufacturer << std::endl;
        std::cout << "  Description : " << description << std::endl;
        std::cout << "  Serial Id   : " << serial << std::endl;
      }
      std::cout << "Done." << std::endl;
      
      curdev = curdev->next;
    }
  
    ftdi_list_free(&devlist);
    ftdi_deinit(&ftdic);
    return 0;
  }
private:
};
