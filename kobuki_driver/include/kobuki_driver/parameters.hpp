/**
 * @file /cruizcore/include/cruizcore/parameters.hpp
 *
 * @brief Parameter configuration for the cruizecore.
 *
 * @date August 2010
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
#include <vector>
#include <ecl/devices/serial.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Parameter configuration for the cruizecore.
 *
 * The following parameters must be configured.
 *
 * - device_port : for serial device, a port (e.g. "/dev/ttyUSB0").
 * - sigslots_namespace : this should match the kobuki_node namespace.
 * - protocol version : firmware version number (e.g. "2.0")
 * - simulation : whether to put the motors on loopback or not.
 */
class Parameters
{
public:
  Parameters() : simulation(false) {}

  std::string device_port;
  std::string protocol_version;
  std::string sigslots_namespace;
  bool simulation;

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

#endif /* YCS_ROBOT_KOBUKI_PARAMETERS_HPP_ */
