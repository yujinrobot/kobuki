/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
    uint8_t bump;
    uint8_t wheel_drop;
    uint8_t cliff;
    uint16_t left_encoder;
    uint16_t right_encoder;
    char left_pwm;
    char right_pwm;
    uint8_t buttons;
    uint8_t charger;
    uint8_t battery;
  } data;

  struct Flags {
    // buttons
    static const uint8_t F0 = 0x02;
    static const uint8_t F1 = 0x01;
    static const uint8_t F2 = 0x04;
    // bumper
    static const uint8_t LeftBumper = 0x04;
    static const uint8_t CentreBumper = 0x02;
    static const uint8_t RightBumper = 0x01;

    // Byte "charger" format:
    // - first four bits distinguish between adapter or docking base charging
    static const uint8_t Adapter     = 0x10;
    // - last 4 bits specified the charging status
    static const uint8_t Discharging = 0x00;
    static const uint8_t Charged     = 0x02;
    static const uint8_t Charging    = 0x06;
  };

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace kobuki

#endif /* KOBUKI_CORE_SENSORS_HPP__ */
