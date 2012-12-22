#include <iostream>
#include <vector>
#include <usb.h>

std::vector<struct usb_device *> find_devices(uint16_t vendor, uint16_t product)
{
  struct usb_bus *bus;
  struct usb_device *dev;
  struct usb_bus *busses;
  std::vector<struct usb_device *> ret_vec;

  usb_init();
  usb_find_busses();
  usb_find_devices();
  busses = usb_get_busses();

  for (bus = busses; bus; bus = bus->next)
    for (dev = bus->devices; dev; dev = dev->next)
      if ((dev->descriptor.idVendor == vendor) && (dev->descriptor.idProduct == product))
        ret_vec.push_back(dev);

  return ret_vec;
}

int main(int argc, char** argv)
{
  std::vector<struct usb_device *> devices;

  devices = find_devices(0x0403,0x6001);
  if (devices.empty()) {
    std::cout << "not found!!!" << std::endl;
    return -1;
  }
  std::cout << devices.size() << " device(s) found." << std::endl;

  int ret_val=0;
  for( unsigned int i=0; i<devices.size(); i++ ) 
  {
    std::cout << std::endl;
    std::cout << "Device #" << i  << std::endl;

    struct usb_device *dev = devices[i];
    usb_dev_handle *h = usb_open(dev);
    if( h < 0 ) {
      std::cerr << "failed to open usb device." << std::endl;
      std::cerr << "do with sudo." << std::endl;
    }
  
    char buff[128];
    int n;
    n = usb_get_string_simple(h, dev->descriptor.iManufacturer, buff, 128);
    if (n < 0) {
      std::cerr << "something is going wrong. do it again with sudo." << std::endl;
      ret_val = -1;
      continue;
    }
    std::cout << "  Manufacturer : " << std::string(buff) << std::endl;
  
    n = usb_get_string_simple(h, dev->descriptor.iProduct, buff, 128);
    if (n < 0) {
      std::cerr << "something is going wrong. do it again with sudo." << std::endl;
      ret_val = -1;
      continue;
    }
    std::cout << "  Product      : " << std::string(buff) << std::endl;

    n = usb_get_string_simple(h, dev->descriptor.iSerialNumber, buff, 128);
    if (n < 0) { 
      std::cerr << "something is going wrong. do it again with sudo." << std::endl;
      ret_val = -1;
      continue;
    }
    std::cout << "  Serial Number: " << std::string(buff) << std::endl;
  } 
  return ret_val;
}  
