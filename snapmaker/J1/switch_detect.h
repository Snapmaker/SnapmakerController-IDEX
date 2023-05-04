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

#include "stdint.h"

class SwitchDetect
{
public:
  void init();
  void init_probe();
  void check();
  void disable_all();
  void enable_filament();
  void disable_filament();
  void trun_on_probe_pwr() {WRITE(PROBE_POWER_EN_PIN, 1);};
  void trun_off_probe_pwr() {WRITE(PROBE_POWER_EN_PIN, 0);};
  void enable_probe(bool trigger_level=0);
  void disable_probe();
  void enable_power_lost();
  void disable_power_lost();
  void enable_stall_guard();
  void disable_stall_guard();
  // void manual_trig_stop();
  // void stall_guard_stop();
  bool read_e0_probe_status();
  bool read_e1_probe_status();
  bool read_active_extruder_status();
  // bool test_trigger();

  bool debug_probe_poweron_sw = true;

private:
  void enable(uint8_t Item);
  void disable(uint8_t Item);

private:
  uint32_t enable_bits;
  uint32_t status_bits;
  uint8_t probe_detect_level = 0;
};

extern SwitchDetect switch_detect;
