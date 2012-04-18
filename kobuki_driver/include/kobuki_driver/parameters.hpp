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
 * @file /include/kobuki_driver/parameters.hpp
 *
 * @brief Parameter configuration for the kobuki.
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_PARAMETERS_HPP_
#define KOBUKI_PARAMETERS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include "modules/battery.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Parameter list and validator for the kobuki.
 */
class Parameters
{
public:
  Parameters() :
    battery_capacity(Battery::capacity),
    battery_low(Battery::low),
    battery_dangerous(Battery::dangerous)
  {
  }

  std::string device_port;         /**< For the serial device, a port (e.g. "/dev/ttyUSB0") **/
  std::string protocol_version;    /**< firmware version number (e.g. '2.0') **/
  std::string sigslots_namespace;  /**< this should match the kobuki-node namespace **/
  bool simulation;                 /**< whether to put the motors in loopback mode or not **/
  double battery_capacity;         /**< Capacity voltage of the battery **/
  double battery_low;              /**< Low level warning for battery level. **/
  double battery_dangerous;        /**< Battery in imminent danger of running out. **/

  /**
   * @brief This is a very rough validator for input configurations.
   *
   * This validates the current parameters and if invalid, puts an error string in error_msg.
   * @return bool : true if valid, false otherwise.
   */
  bool validate()
  {
    if (protocol_version != "2.0")
    {
      error_msg = std::string("protocol_version must be '2.0' (") + protocol_version + std::string(")");
      return false;
    }
    return true;
  }

  std::string error_msg;
};

} // namespace kobuki

#endif /* KOBUKI_PARAMETERS_HPP_ */
