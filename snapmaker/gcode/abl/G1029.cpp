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
#include "../../J1/calibration.h"

enum {
  CALIBRATION_Z_OFFSET, // 0
  CALIBRATION_NOZZLE_HEIGHT, // 1
  CALIBRATION_DED_LEVEL, // 2
  CALIBRATION_XY, //3
  CALIBRATION_PLATE_THICKNESS, // 4
};

void GcodeSuite::G1029() {
  if(parser.seenval('S')) {
    uint8_t set_item;
    set_item = parser.value_byte();
    switch(set_item) {
      case CALIBRATION_Z_OFFSET:
        calibration.calibrate_z_offset();
        break;
      case CALIBRATION_NOZZLE_HEIGHT:
        calibration.calibrate_nozzle_height();
        break;
      case CALIBRATION_DED_LEVEL:
        calibration.calibrate_platform();
        break;
      case CALIBRATION_XY:
        calibration.calibrate_xy();
      case CALIBRATION_PLATE_THICKNESS:
        if(parser.seenval('F')) {
          calibration.set_build_plate_thickness(parser.value_float());
        }
        break;
    }
  } else if(parser.seenval('P')) {
    uint8_t number = parser.value_byte();
    calibration.calibrate_move_xy(number);
  }
}
