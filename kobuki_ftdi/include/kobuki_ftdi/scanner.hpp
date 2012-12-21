#include <iostream>
#include <usb.h>

static struct usb_device *find_devices(uint16_t vendor, uint16_t product)
{
  struct usb_bus *bus;
  struct usb_device *dev;
  struct usb_bus *busses;

  usb_init();
  usb_find_busses();
  usb_find_devices();
  busses = usb_get_busses();

  for (bus = busses; bus; bus = bus->next)
    for (dev = bus->devices; dev; dev = dev->next)
      if ((dev->descriptor.idVendor == vendor) && (dev->descriptor.idProduct == product))
        return dev;

  return NULL;
}

class FTDI_Scanner {
public:
  int scan()
  {
    //struct usb_device *dev;
    dev = NULL;
    dev = find_devices(0x0403,0x6001);
  
    if( dev == NULL ) {
      std::cout << "not found." << std::endl;
      return -1;
    }
    std::cout << "found!!!" << std::endl;
    return 0;
    
  }
  
  int get_serial_id(std::string &serial_id)
  {
    char buff[128];
    // usb_get_string_descriptor_ascii( dev, dev->descriptor.iSerialNumber, buff, 128);
    usb_dev_handle *h = usb_open(dev);
    if( h < 0 ) return -1;
    //  std::cout << h << std::endl;
    //  std::cout << (int)dev->descriptor.iSerialNumber << std::endl;
    int n = usb_get_string_simple(h, dev->descriptor.iSerialNumber, buff, 128);
    if( n  > 0 ) {
      serial_id = std::string(buff);
      std::cout << n << ": " << serial_id << std::endl;
      return 0;
    }
    return -2;
  }

  int reset()
  {
    usb_dev_handle *h = usb_open(dev);
    if( h < 0 ) {
      std::cout << "file open failed." << std::endl;
      return -1;
    }
    int ret_val = usb_reset(h);
    return ret_val;
  }
private:
    struct usb_device *dev;
};
