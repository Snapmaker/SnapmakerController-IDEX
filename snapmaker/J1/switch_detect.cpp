/*
 * Snapmaker 3D Printer Firmware
 * Copyright (C) 2023 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of SnapmakerController-IDEX
 * (see https://github.com/Snapmaker/SnapmakerController-IDEX)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "src/inc/MarlinConfigPre.h"
#include "src/core/millis_t.h"
#include "HAL.h"
#include "src/module/motion.h"
#include "src/module/temperature.h"
#include "src/module/planner.h"
#include "src/module/stepper.h"
#include "switch_detect.h"
#include "../module/filament_sensor.h"
#include "src/module/endstops.h"
#include "../module/motion_control.h"

#define SW_FILAMNET0_BIT      0
#define SW_FILAMNET1_BIT      1
#define SW_PROBE0_BIT         2
#define SW_PROBE1_BIT         3
#define SW_POWER_LOSS_BIT     4
#define SW_STALL_GUARD_BIT    5
#define SW_MANUAL_STOP_BIT    6
#define SW_STALLGUARD_BIT     7

SwitchDetect switch_detect;

#define UPDATE_STATUS(IO, level, reg, bit) do{ \
                                      if(TEST(enable_bits, bit)) { \
                                        if(READ(IO) == level) SBI(reg, bit); \
                                        else CBI(reg, bit); \
                                        if(TEST(reg, bit) && TEST(status_bits, bit)) \
                                          trigged = true; \
                                      } \
                                    } while(0)

#define UPDATE_STATUS_REG(IO, bit)  UPDATE_STATUS(IO, status_bits, bit)

#define TRIGER

void SwitchDetect::init() {
  SET_OUTPUT(PROBE_POWER_EN_PIN);
  SET_INPUT_PULLUP(STALL_GUARD_PIN);
  init_probe();
  disable_all();
  motion_control.init_stall_guard();
}

void SwitchDetect::init_probe() {
  SET_INPUT_PULLUP(X0_CAL_PIN);
  SET_INPUT_PULLUP(X1_CAL_PIN);
}

void SwitchDetect::check() {

  if (!enable_bits) return;

  uint32_t tmp_status_bits = 0;
  bool trigged = false;

  // if(TEST(status_bits, SW_MANUAL_STOP_BIT)) {
  //   trigged = true;
  //   CBI(status_bits, SW_MANUAL_STOP_BIT);
  // }
  // if(TEST(status_bits, SW_STALLGUARD_BIT)) {
  //   trigged = true;
  //   CBI(status_bits, SW_STALLGUARD_BIT);
  // }

  if (!trigged) {
    uint32_t max_continue_trigger_cnt = 0;
    do {
      if (TEST(enable_bits, SW_PROBE0_BIT)) {
        if (READ(X0_CAL_PIN) == probe_detect_level)
          trigged = true;
        else
          trigged = false;
      }
      max_continue_trigger_cnt++;
    } while(trigged && max_continue_trigger_cnt < 10);
  }

  if (!trigged) {
    uint32_t max_continue_trigger_cnt = 0;
    do {
      if (TEST(enable_bits, SW_PROBE1_BIT)) {
        if (READ(X1_CAL_PIN) == probe_detect_level)
          trigged = true;
        else
          trigged = false;
      }
      max_continue_trigger_cnt++;
    } while(trigged && max_continue_trigger_cnt < 10);
  }

  if(trigged == true)
    stepper.quick_stop();

  status_bits = tmp_status_bits;
}

void SwitchDetect::disable_all() {
  enable_bits = 0;
  status_bits = 0;
}

void SwitchDetect::enable_probe(bool trigger_level) {
  init_probe();
  probe_detect_level = trigger_level;
  enable(SW_PROBE0_BIT);
  enable(SW_PROBE1_BIT);
}

void SwitchDetect::disable_probe() {
  disable(SW_PROBE0_BIT);
  disable(SW_PROBE1_BIT);
}

void SwitchDetect::enable_power_lost() {
  enable(SW_POWER_LOSS_BIT);
}

void SwitchDetect::disable_power_lost() {
  disable(SW_POWER_LOSS_BIT);
}

void SwitchDetect::enable_stall_guard() {
  enable(SW_STALL_GUARD_BIT);
}

void SwitchDetect::disable_stall_guard() {
  disable(SW_STALL_GUARD_BIT);
}

void SwitchDetect::enable(uint8_t Item) {
  SBI(enable_bits, Item);
  CBI(status_bits, Item);
}

void SwitchDetect::disable(uint8_t Item) {
  CBI(enable_bits, Item);
  CBI(status_bits, Item);
}

// void SwitchDetect::manual_trig_stop() {
//   SBI(status_bits, SW_MANUAL_STOP_BIT);
// }

// void SwitchDetect::stall_guard_stop() {
//   SBI(status_bits, SW_STALLGUARD_BIT);
// }

bool get_cal_pin_status(uint8_t pin) {
  // bool ret = 0;
  // if (!READ(PROBE_POWER_EN_PIN)) {
  //   if (switch_detect.debug_probe_poweron_sw)
  //     WRITE(PROBE_POWER_EN_PIN, 1);
  //   vTaskDelay(pdMS_TO_TICKS(1));
  //   ret = READ(pin) == LOW;
  //   WRITE(PROBE_POWER_EN_PIN, 0);
  // } else {
  //   ret = READ(pin) == LOW;
  // }
  // return ret;
  return (READ(pin) == LOW);
}

bool SwitchDetect::read_e0_probe_status() {
  return get_cal_pin_status(X0_CAL_PIN);
}

bool SwitchDetect::read_e1_probe_status() {
  return get_cal_pin_status(X1_CAL_PIN);
}

bool SwitchDetect::read_active_extruder_status() {
  return  active_extruder ?
          READ(X1_CAL_PIN) == probe_detect_level :
          READ(X0_CAL_PIN) == probe_detect_level;
}

// bool SwitchDetect::test_trigger() {
//   // bool trigged = false;

//   if(TEST(status_bits, SW_MANUAL_STOP_BIT)) {
//     // trigged = true;
//     return true;
//   }
//   if(TEST(status_bits, SW_STALLGUARD_BIT)) {
//     // trigged = true;
//     return true;
//   }

//   if (TEST(enable_bits, SW_PROBE0_BIT) && (READ(X0_CAL_PIN) == probe_detect_level))
//     // trigged = true;
//     return true;

//   if (TEST(enable_bits, SW_PROBE1_BIT) && (READ(X1_CAL_PIN) == probe_detect_level))
//     // trigged = true;
//     return true;

//   // return trigged;
//   return false;
// }

