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

#include "../../event/event.h"
#include "../../debug/debug.h"
#include "../../../Marlin/src/gcode/gcode.h"

/**
 *  S5 P0/1
 */
void GcodeSuite::M2000() {
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0xFF);
  switch (s) {
    case 1:
      debug.show_all_status();
      break;
    case 2:
      WRITE(E0_AUTO_FAN_PIN, 1);
      break;
    case 3:
      WRITE(E1_AUTO_FAN_PIN, 1);
      break;
    case 4:
      WRITE(CHAMBER_AUTO_FAN_PIN, 1);
      break;
    case 5:
      { // set pc protocol
        event_handler.recv_enable(EVENT_SOURCE_MARLIN);
      }
      break;
    case 6:
      WRITE(MOTOR_PWR_PIN, 1);
      WRITE(HEATER_PWR_PIN, 1);
      WRITE(HEATER_BED_PWR_PIN, 1);
      break;
    case 7:
      WRITE(MOTOR_PWR_PIN, 0);
      WRITE(HEATER_PWR_PIN, 0);
      WRITE(HEATER_BED_PWR_PIN, 0);
      break;
    default:
      break;
  }
}

