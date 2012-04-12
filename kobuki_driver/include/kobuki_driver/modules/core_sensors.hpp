/**
 * @file /include/kobuki_driver/modules/core_sensors.hpp
 *
 * Module for handling of core sensor packet payloads.
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_CORE_SENSORS_HPP__
#define KOBUKI_CORE_SENSORS_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class CoreSensors : public packet_handler::payloadBase
{
public:
  struct Data {
    uint16_t time_stamp;
    unsigned char bump;
    unsigned char wheel_drop;
    unsigned char cliff;
    uint16_t left_encoder;
    uint16_t right_encoder;
    char left_pwm;
    char right_pwm;
    unsigned char buttons;
    unsigned char charger;
    unsigned char battery;
  } data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace kobuki

#endif /* KOBUKI_CORE_SENSORS_HPP__ */
