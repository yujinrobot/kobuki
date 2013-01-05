/**
 * @file include/kobuki_driver/parameters.hpp
 *
 * @brief Parameter configuration for the kobuki.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
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
    simulation(false),
    enable_gate_keeper(true),
    battery_capacity(Battery::capacity),
    battery_low(Battery::low),
    battery_dangerous(Battery::dangerous)
  {
  }

  std::string device_port;         /**< For the serial device, a port (e.g. "/dev/ttyUSB0") **/
  std::string sigslots_namespace;  /**< this should match the kobuki-node namespace **/
  bool simulation;                 /**< whether to put the motors in loopback mode or not **/
  bool enable_gate_keeper;
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
    // not doing anything right now -  delete it, if we can find a use case ...
    return true;
  }

  std::string error_msg;
};

} // namespace kobuki

#endif /* KOBUKI_PARAMETERS_HPP_ */
