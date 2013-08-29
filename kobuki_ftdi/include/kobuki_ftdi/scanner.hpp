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
 * @file include/kobuki_ftdi/scanner.hpp
 * @brief Scan the usb devices, and retrieve its serial id and else descriptions, especially for ftdi devices.
 *
 * Product ID and Vendor ID is fixed to FTDI FT232R chip. If you modify these value, you can use another ftdi device or USB devices.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <vector>
#include <map>
#include <usb.h>
#include <stdint.h>

/*****************************************************************************
 ** Definition
 *****************************************************************************/

//! A Scanner class
/*!
 A Scanner class for FTDI device.
 */
class FTDI_Scanner {
public:
  /**
   * find_devices with given vendor and device id  by using libusb library
   * @param [in] vendor vendor id of targeted usb device
   * @param [in] product product id of targeted usb device
   * @return vector of usb_device structure found.
   **/
  std::vector<struct usb_device *> find_devices(uint16_t vendor, uint16_t product)
  {
    struct usb_bus *bus;
    struct usb_device *dev;
    struct usb_bus *busses;
    std::vector<struct usb_device *> ret_vec;

    usb_init();
    //usb_set_debug(32); // disable it to see bebug outputs
    usb_find_busses();
    usb_find_devices();
    busses = usb_get_busses();

    for (bus = busses; bus; bus = bus->next)
      for (dev = bus->devices; dev; dev = dev->next)
        if ((dev->descriptor.idVendor == vendor) && (dev->descriptor.idProduct == product))
          ret_vec.push_back(dev);

    return ret_vec;
  }

  /**
   * Scan the usb device that have given vendor and product id.
   * @return positive number of devices connected.<br>-1 on failure
   **/
  int scan()
  {
    devices = find_devices(0x0403,0x6001);

    if( devices.empty() ) return -1;
    scanned = true;
    retrieved = false;
    return devices.size();
  }

  /**
   * Retrieve descriptions of scanned devices. Retrived desctiptions are stored internally.
   * @return positive number of retrieved devices.<br>-1 on failure
   * @sa get_serial_id(unsigned int, std::string&), get_serial_id(std::string&)
   * , get_manufacturer(unsigned int, std::string&), get_manufacturer(std::string&)
   * , get_product(unsigned int, std::string&), get_product(std::string&)
   **/
  int retrieve()
  {
    if( !scanned ) if ( scan() < 0 ) return -1;
    if( devices.empty() ) return -1;

    char buff[128];
    descriptions.clear();

    for (unsigned int i=0; i<devices.size(); i++) {
      struct usb_device *dev = devices[i];
      usb_dev_handle *h = usb_open(dev);
      if ( h < 0 ) continue;

      std::map<std::string, std::string> M_desc;
      if ( usb_get_string_simple(h, dev->descriptor.iSerialNumber, buff, 128) < 0 ) continue;
      M_desc["serial_number"] = std::string(buff);

      if ( usb_get_string_simple(h, dev->descriptor.iManufacturer, buff, 128) < 0 ) continue;
      M_desc["manufacturer"] = std::string(buff);

      if ( usb_get_string_simple(h, dev->descriptor.iProduct, buff, 128) < 0 ) continue;
      M_desc["product"] = std::string(buff);

      descriptions.push_back(M_desc);
    }
    retrieved = true;
    return descriptions.size();
  }

  /**
   * @param [in] index index of scanned device
   * @param [out] serial_id serial number of ftdi device
   * @return 0 on success<br>-1 on failure
   * @sa get_serial_id(unsigned int, std::string&), get_serial_id(std::string&)
   * , get_manufacturer(unsigned int, std::string&), get_manufacturer(std::string&)
   * , get_product(unsigned int, std::string&), get_product(std::string&)
   **/
  int get_serial_id(unsigned int index, std::string &serial_id) {
    if( ! retrieved ) if( retrieve() < 0 ) return -1;
    if (descriptions.size() <= index) return -1;
    serial_id = descriptions[index]["serial_number"];
    return 0;
  }

  /**
   * @param [in] index index of scanned device
   * @param [out] manufacturer manufacturer name of ftdi device
   * @return 0 on success<br>-1 on failure
   * @sa get_serial_id(unsigned int, std::string&), get_serial_id(std::string&)
   * , get_manufacturer(unsigned int, std::string&), get_manufacturer(std::string&)
   * , get_product(unsigned int, std::string&), get_product(std::string&)
   **/
  int get_manufacturer (unsigned int index, std::string &manufacturer) {
    if( ! retrieved ) if( retrieve() < 0 ) return -1;
    if (descriptions.size() <= index) return -1;
    manufacturer = descriptions[index]["manufacturer"];
    return 0;
  }

  /**
   * @param [in] index index of scanned device
   * @param [out] product product name of ftdi device
   * @return 0 on success<br>-1 on failure
   * @sa get_serial_id(unsigned int, std::string&), get_serial_id(std::string&)
   * , get_manufacturer(unsigned int, std::string&), get_manufacturer(std::string&)
   * , get_product(unsigned int, std::string&), get_product(std::string&)
   **/
  int get_product (unsigned int index, std::string &product) {
    if( ! retrieved ) if( retrieve() < 0 ) return -1;
    if (descriptions.size() <= index) return -1;
    product = descriptions[index]["product"];
    return 0;
  }

  /**
   * @param [out] serial_id serial number of ftdi device
   * @return 0 on success<br>-1 on failure
   * @sa get_serial_id(unsigned int, std::string&), get_serial_id(std::string&)
   * , get_manufacturer(unsigned int, std::string&), get_manufacturer(std::string&)
   * , get_product(unsigned int, std::string&), get_product(std::string&)
   **/
  int get_serial_id(std::string &serial_id) { return get_serial_id(0, serial_id); }

  /**
   * @param [out] manufacturer manufacturer name of of ftdi device
   * @return 0 on success<br>-1 on failure
   * @sa get_serial_id(unsigned int, std::string&), get_serial_id(std::string&)
   * , get_manufacturer(unsigned int, std::string&), get_manufacturer(std::string&)
   * , get_product(unsigned int, std::string&), get_product(std::string&)
   **/
  int get_manufacturer (std::string &manufacturer) { return get_manufacturer(0, manufacturer); }

  /**
   * @param [out] product product name of of ftdi device
   * @return 0 on success<br>-1 on failure
   * @sa get_serial_id(unsigned int, std::string&), get_serial_id(std::string&)
   * , get_manufacturer(unsigned int, std::string&), get_manufacturer(std::string&)
   * , get_product(unsigned int, std::string&), get_product(std::string&)
   **/
  int get_product (std::string &product) { return get_product(0, product); }

  /**
   * reset the connected usb device to recall it to standard tty interface
   * @return 0 on success<br>-1 on failure
   **/
  int reset()
  {
    if( devices.empty() ) {
      if( scan() < 0 ) return -1;
    };

    struct usb_device *dev = devices[0];
    usb_dev_handle *h = usb_open(dev);
    if( h < 0 ) {
      return -1;
    }
    int ret_val = usb_reset(h);
    return ret_val;
  }

private:
    bool scanned; /**< flags to indicated current status of calss */
    bool retrieved; /**< flags to indicated current status of calss */
    std::vector<struct usb_device *> devices; /**< Container for data structure of  usb devices found*/
    std::vector<std::map<std::string, std::string> > descriptions; /**< Container for usb data of devices found. such as serial_number, product name, manufacture name*/
};
