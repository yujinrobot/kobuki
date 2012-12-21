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
  return 0;
}
