/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfig.h"

#if HAS_TRINAMIC_CONFIG

#include "../../gcode.h"
#include "../../../feature/tmc_util.h"
#include "../../../module/stepper/indirection.h"
#include "../../../../snapmaker/module/motion_control.h"
/**
 * M122: Debug TMC drivers
 */
void GcodeSuite::M122() {
  xyze_bool_t print_axis = ARRAY_N_1(LOGICAL_AXES, false);

  bool print_all = true;
  LOOP_LOGICAL_AXES(i) if (parser.seen_test(axis_codes[i])) { print_axis[i] = true; print_all = false; }

  if (print_all) LOOP_LOGICAL_AXES(i) print_axis[i] = true;

  if (parser.seenval('S')) {
    uint8_t s = parser.value_byte();
    uint32_t p = 0;
    uint32_t e = 2;
    if (parser.seenval('P')) {
      p = parser.value_int();
    }
    if (parser.seenval('I')) {
      e = parser.value_int();
    }
    LOOP_LOGICAL_AXES(i) {
      if (print_axis[i]) {
        if (s) {
          motion_control.enable_stall_guard(i, p, e);
        } else {
          motion_control.disable_stall_guard(i);
        }
      }
    }
    return;
  }
  if (parser.seen('G')) {
    LOOP_LOGICAL_AXES(i) {
      if (print_axis[i]) {
        report_sgthrs(i);
        tmc_report_sg_result(i);
      }
    }
    return;
  }
  if (parser.seen('W')) {
    uint8_t thrs = parser.value_byte();
    LOOP_LOGICAL_AXES(i) {
      if (print_axis[i]) {
        set_sgthrs(i, thrs);
        tmc_report_sg_result(i);
      }
    }
    return;
  }


  if (parser.boolval('I')) restore_stepper_drivers();

  #if ENABLED(TMC_DEBUG)
    #if ENABLED(MONITOR_DRIVER_STATUS)
      uint16_t interval = MONITOR_DRIVER_STATUS_INTERVAL_MS;
      if (parser.seen('S') && !parser.value_bool()) interval = 0;
      if (parser.seenval('P')) NOMORE(interval, parser.value_ushort());
      tmc_set_report_interval(interval);
    #endif

    if (parser.seen_test('V'))
      tmc_get_registers(LOGICAL_AXIS_ELEM(print_axis));
    else
      tmc_report_all(LOGICAL_AXIS_ELEM(print_axis));
  #endif

  test_tmc_connection(LOGICAL_AXIS_ELEM(print_axis));
}

#endif // HAS_TRINAMIC_CONFIG
