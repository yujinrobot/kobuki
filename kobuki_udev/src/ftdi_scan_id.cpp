/**
 * @file /ftdi/src/utils/ftdi_scan_id.cpp
 *
 * @brief Scan existing ftdi chips and print out their serial id strings.
 *
 * @date August 2010
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <ftdi.h>

/*****************************************************************************
 ** Main
 *****************************************************************************/

int main(int argc, char **argv)
{

  int ret, i, no_devices;
  struct ftdi_context ftdic;
  struct ftdi_device_list *devlist, *curdev;
  char manufacturer[128], description[128];
  char serial[128];

  if (ftdi_init(&ftdic) < 0)
  {
    std::cerr << "ftdi_init failed" << std::endl;
    return EXIT_FAILURE;
  }

  if ((no_devices = ftdi_usb_find_all(&ftdic, &devlist, 0x0403, 0x6001)) < 0)
  {
    std::cerr << "ftdi_usb_find_all failed: " << ftdi_get_error_string(&ftdic) << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Number of FTDI devices found: " << no_devices << std::endl;

  i = 0;
  for (curdev = devlist; curdev != NULL; i++)
  {
    std::cout << "Device #" << i << std::endl;
    if ((ret = ftdi_usb_get_strings(&ftdic, curdev->dev, manufacturer, 128, description, 128, serial, 128)) < 0)
    {
      std::cerr << "ftdi_usb_get_strings failed: " << ftdi_get_error_string(&ftdic) << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "  Manufacturer: " << manufacturer << std::endl;
    std::cout << "  Description : " << description << std::endl;
    std::cout << "  Serial Id   : " << serial << std::endl;
    curdev = curdev->next;
  }

  ftdi_list_free(&devlist);
  ftdi_deinit(&ftdic);

  return 0;
}

