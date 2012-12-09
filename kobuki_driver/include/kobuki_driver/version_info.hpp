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
 * @file /kobuki_driver/include/kobuki_driver/version.hpp
 *
 * @brief Version info for the kobuki driver.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_VERSION_HPP_
#define KOBUKI_VERSION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <stdint.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/
/**
 * Class holding version info for the kobuki driver.
 */
class VersionInfo {
public:
  VersionInfo(const uint32_t &fw, const uint32_t &hw, const uint32_t udid0_, const uint32_t udid1_, const uint32_t udid2_ ) :
    firmware(fw),
    hardware(hw),
    software(0),
    udid0(udid0_),
    udid1(udid1_),
    udid2(udid2_)
  {}
  const uint32_t firmware;
  const uint32_t hardware;
  const uint32_t software;
  const uint32_t udid0;
  const uint32_t udid1;
  const uint32_t udid2;

  static std::string toString(uint32_t version)
  {
    // Convert an unsigned int into a string of type <mayor>.<minor>.<patch>; first byte is ignored
    std::stringstream ss;
    ss << ((version & 0x00FF0000) >> 16) << "." << ((version & 0x0000FF00) >>  8) << "." << (version & 0x000000FF);

    return std::string(ss.str());
  }
};

} // namespace kobuki
#endif /* KOBUKI_VERSION_HPP_ */
