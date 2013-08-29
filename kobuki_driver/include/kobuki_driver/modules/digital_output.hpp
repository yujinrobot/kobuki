/**
 * @file /kobuki_driver/include/kobuki_driver/modules/digital_output.hpp
 *
 * @brief Digital output flags.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_DIGITAL_OUTPUT_HPP_
#define KOBUKI_DIGITAL_OUTPUT_HPP_

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Structures
 *****************************************************************************/
/**
 * Convenient class for setting values for digital output pins.
 */
struct DigitalOutput {
  DigitalOutput() {
    for ( unsigned int i = 0; i < 4; ++i ) {
      values[i] = false;
      mask[i] = false;
    }
  }
  bool values[4]; /**< Digital on or off for pins 0-3 respectively. **/
  bool mask[4]; /**< Set indices to true to set a pin, false to ignore. **/
};


} // namespace kobuki

#endif /* KOBUKI_DIGITAL_OUTPUT_HPP_ */
