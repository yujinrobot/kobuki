/**
 * @file include/kobuki_driver/packets/core_sensors.hpp
 *
 * @brief Core sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
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
#include <stdint.h>

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
    uint8_t bumper;
    uint8_t wheel_drop;
    uint8_t cliff;
    uint16_t left_encoder;
    uint16_t right_encoder;
    char left_pwm;
    char right_pwm;
    uint8_t buttons;
    uint8_t charger;
    uint8_t battery;
    uint8_t over_current;
  } data;

  struct Flags {
    // buttons
    static const uint8_t Button0 = 0x01;
    static const uint8_t Button1 = 0x02;
    static const uint8_t Button2 = 0x04;

    // bumper
    static const uint8_t LeftBumper   = 0x04;
    static const uint8_t CenterBumper = 0x02;
    static const uint8_t RightBumper  = 0x01;

    // cliff sensor
    static const uint8_t LeftCliff    = 0x04;
    static const uint8_t CenterCliff  = 0x02;
    static const uint8_t RightCliff   = 0x01;

    // wheel drop sensor
    static const uint8_t LeftWheel    = 0x02;
    static const uint8_t RightWheel   = 0x01;

    // Charging source
    // - first four bits distinguish between adapter or docking base charging
    static const uint8_t AdapterType  = 0x10;
    // - last 4 bits specified the charging status (see Battery.hpp for details)
    static const uint8_t BatteryStateMask = 0x0F;
    static const uint8_t Discharging  = 0x00;
    static const uint8_t Charged      = 0x02;
    static const uint8_t Charging     = 0x06;

    // wheel drop sensor
    static const uint8_t LeftWheel_OC    = 0x01;
    static const uint8_t RightWheel_OC   = 0x02;

  };

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace kobuki

#endif /* KOBUKI_CORE_SENSORS_HPP__ */
