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
 * @file src/overwrite_serial_number.cpp
 * @brief Overwrite serial number of ftdi device.
 *
 * <b>License:</b> BSD https://raw.github.com/yujinrobot/kobuki/master/kobuki_node/LICENSE
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream>
#include <ecl/command_line.hpp>
#include "kobuki_ftdi/writer.hpp"
#include "kobuki_ftdi/scanner.hpp"

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
  /*********************
   ** Parse Command Line
   **********************/
  CmdLine cmd_line("This is used to write a new serial id string to the ftdi device.", ' ', "0.1");
  ValueArg<std::string> old_arg(
      "o", "old", "Identify the device via the old serial id (if there are multiple devices attached) ['unspecified'].", false,
      "unspecified", "string");
  UnlabeledValueArg<std::string> new_arg("new_id", "New serial id used to identify the device [xxx].", true,
                                         "xxx", "string");
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
   ** Writing serial id
   **********************/
  FTDI_Writer writer;
  int ret_val = 0;
  ret_val = writer.write(new_id);
  if (ret_val<0) {
    std::cerr << "Something went wrong." << std::endl;
    return ret_val;
  }

  FTDI_Scanner scanner;
  ret_val = scanner.reset();
  if (ret_val<0 && ret_val !=-19) {
    std::cerr << "Something went wrong." << std::endl;
    return ret_val;
  }

  std::cout << "ret_val: " << ret_val << std::endl;
  return 0;
}

