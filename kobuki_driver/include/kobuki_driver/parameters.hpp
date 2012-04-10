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
  Parameters() : simulation(false) {}

  std::string device_port;         /**< For the serial device, a port (e.g. "/dev/ttyUSB0"). **/
  std::string protocol_version;    /**< firmware version number (e.g. '2.0') **/
  std::string sigslots_namespace;  /**< this should match the kobuki-node namespace **/
  bool simulation;                 /**< whether to put the motors in loopback mode or not. **/

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
