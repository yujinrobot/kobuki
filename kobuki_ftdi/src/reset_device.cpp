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

int main(int argc, char** argv)
{
  std::cout << "hello world." << std::endl;

  struct usb_device *dev;
  dev = find_devices(0x0403,0x6001);

  if( dev == NULL ) {
    std::cout << "there are no devices to reset." << std::endl;
    return -1;
  }
  std::cout << "found!!!" << std::endl;

  usb_dev_handle *h = usb_open(dev);
  if( h < 0 ) {
    std::cout << "failed to open usb device. do it again with sudo." << std::endl;
    return -1;
  }

  int ret_val = usb_reset( h );
  if( ret_val < 0 ) {
    std::cout << ret_val << ": something is going wrong." << std::endl;
    return -1;
  }

  return 0;
}
