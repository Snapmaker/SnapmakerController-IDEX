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

/**
 * G1029.cpp - Auto calibration
 */
#include "../../../Marlin/src/gcode/gcode.h"
// #include "../../J1/calibration.h"
#include "../../module/calibtration.h"
#include "../../module/motion_control.h"
#include "../../module/fdm.h"


void GcodeSuite::G1029() {
  if(parser.seenval('I')) {
    uint8_t number = parser.value_byte();
    calibtration.goto_calibtration_position(number);
  } else if (parser.seenval('B')) {
    uint8_t number = parser.value_byte();
    calibtration.bed_calibtration_preapare((calibtration_position_e)number, number==CAlIBRATION_POS_1);
    if (number != CAlIBRATION_POS_1) {
      calibtration.bed_start_bead_mode();
    }
  } else if (parser.seenval('N')) {
    uint8_t number = parser.value_byte();
    calibtration.nozzle_calibtration_preapare((calibtration_position_e)number);
    calibtration.bed_start_bead_mode();
  } else if (parser.seen('A')) {
    calibtration.calibtration_xy();
  } else if (parser.seen('E')) {
    calibtration.exit();
  } else if (parser.seen('P')) {
    fdm_head.set_temperature(0, 220);
    fdm_head.set_temperature(1, 220);
    motion_control.move_x_to_relative_home(50);
  }
}
