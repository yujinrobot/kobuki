/**
 * @file /kobuki_driver/include/kobuki_driver/modules/battery.hpp
 *
 * @brief Human friendly batter indicator class.
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_BATTERY_HPP_
#define KOBUKI_BATTERY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief  Battery level module.
 *
 * Currently hard codes the battery status. It might be useful to provide
 * some configurable parameters for this module in the future.
 **/
class Battery {
public:
  Battery () {}; /**< Default constructor. **/

private:
};

} // namespace kobuki

#endif /* KOBUKI_BATTERY_HPP_ */
