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
  VersionInfo(const uint16_t &fw, const uint16_t &hw) :
    firmware(fw),
    hardware(hw),
    software(20120414)
  {}
  const uint16_t firmware;
  const uint16_t hardware;
  const uint32_t software;
};

} // namespace kobuki
#endif /* KOBUKI_VERSION_HPP_ */
