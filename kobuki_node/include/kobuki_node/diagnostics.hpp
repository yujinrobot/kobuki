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
 * @file /kobuki_node/include/kobuki_node/diagnostics.hpp
 *
 * @brief Diagnostics for the kobuki node.
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_NODE_DIAGNOSTICS_HPP_
#define KOBUKI_NODE_DIAGNOSTICS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <kobuki_driver/packets/cliff.hpp>
#include <kobuki_driver/modules/battery.hpp>
#include <kobuki_driver/packets/core_sensors.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/


/**
 * Diagnostic checking the robot battery and charging status.
 */
class BatteryTask : public diagnostic_updater::DiagnosticTask {
public:
  BatteryTask() : DiagnosticTask("Battery") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const Battery &battery) { status = battery; }

private:
  Battery status;
};

/**
 * Simple diagnostic checking to see if kobuki is streaming data or not.
 */
class WatchdogTask : public diagnostic_updater::DiagnosticTask {
public:
  WatchdogTask() : DiagnosticTask("Watchdog"), alive(false) {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const bool &is_alive) { alive = is_alive; }
  bool isAlive() const { return alive; }

private:
  bool alive;
};

/**
 * Diagnostic checking the cliff sensors status.
 */
class CliffSensorTask : public diagnostic_updater::DiagnosticTask {
public:
  CliffSensorTask() : DiagnosticTask("Cliff Sensor") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(uint8_t new_status, const Cliff::Data &new_values) {
    status = new_status; values = new_values;
  }

private:
  uint8_t     status;
  Cliff::Data values;
};

/**
 * Diagnostic checking the wall sensors (aka bumpers) status.
 */
class WallSensorTask : public diagnostic_updater::DiagnosticTask {
public:
  WallSensorTask() : DiagnosticTask("Wall Sensor") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(uint8_t new_status) { status = new_status; }

private:
  uint8_t status;
};

/**
 * Diagnostic checking whether the wheels stay in contact with the ground.
 */
class WheelDropTask : public diagnostic_updater::DiagnosticTask {
public:
  WheelDropTask() : DiagnosticTask("Wheel Drop") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(uint8_t new_status) { status = new_status; }

private:
  uint8_t status;
};

/**
 * Diagnostic checking the current supplied to the motors, what
 * can be useful for detecting whether the robot is blocked.
 */
class MotorCurrentTask : public diagnostic_updater::DiagnosticTask {
public:
  MotorCurrentTask() : DiagnosticTask("Motor Current") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const std::vector<uint8_t> &new_values) { values = new_values; }

private:
  std::vector<uint8_t> values;
};

/**
 * Diagnostic checking the on/off state of the motors
 */
class MotorStateTask : public diagnostic_updater::DiagnosticTask {
public:
  MotorStateTask() : DiagnosticTask("Motor State") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(bool new_state) { state = new_state; };

private:
  bool state;
};

/**
 * Diagnostic checking the gyro sensor status.
 */
class GyroSensorTask : public diagnostic_updater::DiagnosticTask {
public:
  GyroSensorTask() : DiagnosticTask("Gyro Sensor") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(int16_t new_heading) { heading = new_heading; }

private:
  int16_t heading;
};

/**
 * Diagnostic checking the state of the digital input port (four bits).
 */
class DigitalInputTask : public diagnostic_updater::DiagnosticTask {
public:
  DigitalInputTask() : DiagnosticTask("Digital Input") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(uint16_t new_status) { status = new_status; }

private:
  uint16_t status;
};

/**
 * Diagnostic checking the state of the analog input port (four short integers).
 */
class AnalogInputTask : public diagnostic_updater::DiagnosticTask {
public:
  AnalogInputTask() : DiagnosticTask("Analog Input") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const std::vector<uint16_t> &new_values) { values = new_values; }

private:
  std::vector<uint16_t> values;
};

} // namespace kobuki

#endif /* KOBUKI_NODE_DIAGNOSTICS_HPP_ */
