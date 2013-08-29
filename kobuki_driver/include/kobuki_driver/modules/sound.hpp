/**
 * @file /kobuki_driver/include/kobuki_driver/modules/sound.hpp
 *
 * @brief Flags and id's for commanding sound sequences.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_SOUND_HPP_
#define KOBUKI_SOUND_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Enums
*****************************************************************************/

enum SoundSequences
{
  On = 0x0, /**< Turn on **/
  Off = 0x1, /**< Turn off **/
  Recharge = 0x2, /**< Recharging starting **/
  Button = 0x3, /**< Button pressed  **/
  Error = 0x4, /**< Error sound **/
  CleaningStart = 0x5, /**< Cleaning started **/
  CleaningEnd = 0x6, /**< Cleaning ended **/
};

} // namespace kobuki

#endif /* KOBUKI_SOUND_HPP_ */
