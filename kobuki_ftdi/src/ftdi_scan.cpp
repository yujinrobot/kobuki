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
 * @file src/ftdi_scan.cpp
 *
 * @brief Scan existing ftdi chips and print out their serial id strings.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
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

