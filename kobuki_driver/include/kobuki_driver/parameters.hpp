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

namespace kobuki {

/*****************************************************************************
** Interface
*****************************************************************************/
/**
 * @brief Parameter configuration for the cruizecore.
 *
 * The following parameters must be configured.
 *
 * - device_name : must be either "serial" or "ftdi".
 * - device_id : for serial, a port (e.g. "/dev/ttyUSB0"), for ftdi, the serial id.
 * - sigslots_namespace : unique name for the sigslots communications.
 *
 */
class Parameters {
public:
	std::string device_name;
	std::string device_id;
	std::string device_port;
	std::string protocol_version;
	std::string sigslots_namespace;

	/**
	 * @brief This is a very rough validator for input configurations.
	 *
	 * This validates the current parameters and if invalid, puts an error string in error_msg.
	 * @return bool : true if valid, false otherwise.
	 */
	bool validate() {
		if ( ( device_name != "serial") && (device_name != "ftdi") ) {
			error_msg = std::string("name must be either 'serial' or 'ftdi' (") + device_name + std::string(")");
			return false;
		}
		if ( ( protocol_version != "1.0") && (protocol_version != "2.0") ) {
			error_msg = std::string("protocol_version must be either '1.0' or '2.0' (") + protocol_version + std::string(")");
			return false;
		}
		return true;
	}
	std::string error_msg;
};


} // namespace kobuki

#endif /* YCS_ROBOT_KOBUKI_PARAMETERS_HPP_ */
