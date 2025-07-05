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

#if ENABLED(PARK_HEAD_ON_PAUSE)

#include "../../gcode.h"
#include "../../parser.h"
#include "../../../feature/pause.h"
#include "../../../lcd/marlinui.h"
#include "../../../module/motion.h"
#include "../../../module/printcounter.h"
#include "../../../sd/cardreader.h"
#include "../../module/endstops.h"
#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

void GcodeSuite::M125() {
  const float retract = TERN0(HAS_EXTRUDERS, -ABS(parser.axisunitsval('L', E_AXIS, PAUSE_PARK_RETRACT_LENGTH)));
  xyz_pos_t park_point;
  if (active_extruder == 0) {
    park_point = xyz_pos_t{NOZZLE_PARK_POINT_T0};
  } else {
    park_point = xyz_pos_t{NOZZLE_PARK_POINT_T1};
  }

  if (parser.seenval('X')) park_point.x = RAW_X_POSITION(parser.linearval('X'));
  if (parser.seenval('Y')) park_point.y = RAW_X_POSITION(parser.linearval('Y'));
  if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

  #if HAS_HOTEND_OFFSET && NONE(DUAL_X_CARRIAGE, DELTA)
    park_point += hotend_offset[active_extruder];
  #endif

  const bool sd_printing = TERN0(SDSUPPORT, IS_SD_PRINTING());
  ui.pause_show_message(PAUSE_MESSAGE_PARKING, PAUSE_MODE_PAUSE_PRINT);
  const bool show_lcd = TERN0(HAS_LCD_MENU, parser.boolval('P'));

  #if ENABLED(DUAL_X_CARRIAGE)
    if (active_extruder == 1) endstops.enable_globally(false);
  #endif
  if (pause_print(retract, park_point, show_lcd, 0)) {
    if (ENABLED(EXTENSIBLE_UI) || BOTH(EMERGENCY_PARSER, HOST_PROMPT_SUPPORT) || !sd_printing || show_lcd) {
      wait_for_confirmation(false, 0);
      resume_print(0, 0, -retract, 0);
    }
  }
  #if ENABLED(DUAL_X_CARRIAGE)
    if (active_extruder == 1) endstops.enable_globally(true);
  #endif
}

#endif // PARK_HEAD_ON_PAUSE