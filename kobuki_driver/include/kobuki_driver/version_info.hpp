/**
 * @file include/kobuki_driver/version_info.hpp
 *
 * @brief Version info for the kobuki driver.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
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
#include <sstream>
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

  static std::string toString(const uint32_t &version)
  {
    // Convert an unsigned int into a string of type <mayor>.<minor>.<patch>; first byte is ignored
    std::stringstream ss;
    ss << ((version & 0x00FF0000) >> 16) << "." << ((version & 0x0000FF00) >>  8) << "." << (version & 0x000000FF);
    return std::string(ss.str());
  }

  static std::string toString(const uint32_t &udid0, const uint32_t &udid1, const uint32_t &udid2)
  {
    // Convert three udid unsigned integers into a string of type <udid0>-<udid1>-<udid2>
    std::stringstream ss;
    ss << udid0 << "-" << udid1 << "-" << udid2;
    return std::string(ss.str());
  }

  static std::string getSoftwareVersion();
};

} // namespace kobuki
#endif /* KOBUKI_VERSION_HPP_ */
