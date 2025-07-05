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
#include "../../../module/temperature.h"
#include "../../gcode.h"
#include "../../../feature/pause.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/printcounter.h"
#include "../../../lcd/marlinui.h"
#include "../../../module/tool_change.h"
#include "../../../feature/runout.h"
#include "../../../../../snapmaker/debug/debug.h"
#include "../../../../../snapmaker/module/print_control.h"
#include "../../../../../snapmaker/module/power_loss.h"
#include "../../../../../snapmaker/module/filament_sensor.h"
#include "../../../../../snapmaker/module/system.h"
#include "../../module/endstops.h"

void GcodeSuite::M600() {
  if (is_hmi_printing) {
    planner.synchronize();
    power_loss.m600_cur_line = queue.file_line_number();
    LOG_I("power_loss.m600_cur_line set to %d\r\n", power_loss.cur_line);
    system_service.set_status(SYSTEM_STATUE_PAUSING, SYSTEM_STATUE_SCOURCE_M600);
  }

  #if ENABLED(ADVANCED_PAUSE_FEATURE) && !IS_HMI_PRINTING
    const int8_t target_extruder = get_target_extruder_from_command();
    if (target_extruder < 0) return;

    #if ENABLED(DUAL_X_CARRIAGE)
      int8_t DXC_ext = target_extruder;
      if (!parser.seen_test('T')) {
        DXC_ext = filament_sensor.is_trigger(1) ? 1 : 0;
      }
    #else
      #define DXC_ext target_extruder
    #endif

    #if HAS_MULTI_EXTRUDER
      const uint8_t active_extruder_before_filament_change = active_extruder;
      if (active_extruder != target_extruder && TERN1(DUAL_X_CARRIAGE, !idex_is_duplicating()))
        tool_change(target_extruder, false);
    #endif

    const float retract = -ABS(parser.axisunitsval('E', E_AXIS, PAUSE_PARK_RETRACT_LENGTH));
    // Select park point based on target extruder using separate macros
    xyz_pos_t park_point;
    if (target_extruder == 0) {
      park_point = xyz_pos_t{NOZZLE_PARK_POINT_T0};
    } else {
      park_point = xyz_pos_t{NOZZLE_PARK_POINT_T1};
    }
    // Override with command parameters if provided
    if (parser.seenval('Z')) park_point.z = parser.linearval('Z');
    if (parser.seenval('X')) park_point.x = parser.linearval('X');
    if (parser.seenval('Y')) park_point.y = parser.linearval('Y');

    const float unload_length = -ABS(parser.axisunitsval('U', E_AXIS, fc_settings[active_extruder].unload_length));
    constexpr float slow_load_length = FILAMENT_CHANGE_SLOW_LOAD_LENGTH;
    const float fast_load_length = ABS(parser.axisunitsval('L', E_AXIS, fc_settings[active_extruder].load_length));
    const int8_t beep_count = parser.intval('B', -1);

    SERIAL_ECHOLNPAIR("M600: Pausing print for extruder ", target_extruder);
    SERIAL_ECHOLNPAIR("M600: Parking at X=", park_point.x, " Y=", park_point.y, " Z=", park_point.z);
    is_m600_pause = true;

    #if ENABLED(DUAL_X_CARRIAGE)
      if (target_extruder == 1) endstops.enable_globally(false);
    #endif
    if (pause_print(retract, park_point, true, unload_length DXC_PASS)) {
      SERIAL_ECHOLNPGM("M600: Pause succeeded");
      wait_for_confirmation(false, beep_count DXC_PASS);

      SERIAL_ECHOLNPGM("M600: Resuming print");
      resume_print(slow_load_length, fast_load_length, ADVANCED_PAUSE_PURGE_LENGTH,
                   beep_count, (parser.seenval('R') ? parser.value_celsius() : 0) DXC_PASS);
    }
    else {
      SERIAL_ECHOLNPGM("M600: Pause failed");
    }
    #if ENABLED(DUAL_X_CARRIAGE)
      if (target_extruder == 1) endstops.enable_globally(true);
    #endif

    is_m600_pause = false;

    #if HAS_MULTI_EXTRUDER
      if (active_extruder_before_filament_change != active_extruder)
        tool_change(active_extruder_before_filament_change, false);
    #endif
  #endif // ADVANCED_PAUSE_FEATURE && !IS_HMI_PRINTING
}