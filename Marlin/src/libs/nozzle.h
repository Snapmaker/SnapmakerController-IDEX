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
#pragma once

#include "../inc/MarlinConfig.h"

/**
 * @brief Nozzle class
 *
 * @todo: Do not ignore the end.z value and allow XYZ movements
 */
class Nozzle {
  private:

  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    static void stroke(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t &strokes) _Os;
    static void zigzag(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t &strokes, const uint8_t &objects) _Os;
    static void circle(const xyz_pos_t &start, const xyz_pos_t &middle, const uint8_t &strokes, const_float_t radius) _Os;
  #endif

  public:

  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    static void clean(const uint8_t &pattern, const uint8_t &strokes, const_float_t radius, const uint8_t &objects, const uint8_t cleans) _Os;
  #endif

  #if ENABLED(NOZZLE_PARK_FEATURE)
    static float park_mode_0_height(const_float_t park_z) _Os;
    static void park(const uint8_t z_action, const xyz_pos_t &park) _Os;
    static xyz_pos_t get_park_point(uint8_t extruder); // New helper function
  #endif
};

extern Nozzle nozzle;