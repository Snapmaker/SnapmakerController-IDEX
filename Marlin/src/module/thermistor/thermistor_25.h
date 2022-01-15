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

#define REVERSE_TEMP_SENSOR_RANGE_25 1

// #undef OV_SCALE
// #define OV_SCALE(N) (float((N) * 5) / 3.3f)
#define SCALE_HARDWARE_ERROR (0)
constexpr temp_entry_t temptable_25[] PROGMEM = {
  { OV(628  - SCALE_HARDWARE_ERROR), -30 },
  { OV(628  - SCALE_HARDWARE_ERROR), -27 },
  { OV(628  - SCALE_HARDWARE_ERROR), -10 },
  { OV(628  - SCALE_HARDWARE_ERROR),   0 },
  { OV(693  - SCALE_HARDWARE_ERROR),   9 },
  { OV(751  - SCALE_HARDWARE_ERROR),  17 },
  { OV(794  - SCALE_HARDWARE_ERROR),  23 },
  { OV(829  - SCALE_HARDWARE_ERROR),  28 },
  { OV(907  - SCALE_HARDWARE_ERROR),  39 },
  { OV(950  - SCALE_HARDWARE_ERROR),  45 },
  { OV(992  - SCALE_HARDWARE_ERROR),  51 },
  { OV(1020 - SCALE_HARDWARE_ERROR),  55 },
  { OV(1034 - SCALE_HARDWARE_ERROR),  57 },
  { OV(1090 - SCALE_HARDWARE_ERROR),  65 },
  { OV(1117 - SCALE_HARDWARE_ERROR),  69 },
  { OV(1138 - SCALE_HARDWARE_ERROR),  72 },
  { OV(1187 - SCALE_HARDWARE_ERROR),  79 },
  { OV(1215 - SCALE_HARDWARE_ERROR),  85 },
  { OV(1262 - SCALE_HARDWARE_ERROR),  90 },
  { OV(1296 - SCALE_HARDWARE_ERROR),  95 },
  { OV(1330 - SCALE_HARDWARE_ERROR), 100 },
  { OV(1357 - SCALE_HARDWARE_ERROR), 105 },
  { OV(1405 - SCALE_HARDWARE_ERROR), 111 },
  { OV(1445 - SCALE_HARDWARE_ERROR), 117 },
  { OV(1485 - SCALE_HARDWARE_ERROR), 123 },
  { OV(1519 - SCALE_HARDWARE_ERROR), 128 },
  { OV(1552 - SCALE_HARDWARE_ERROR), 133 },
  { OV(1592 - SCALE_HARDWARE_ERROR), 139 },
  { OV(1625 - SCALE_HARDWARE_ERROR), 144 },
  { OV(1664 - SCALE_HARDWARE_ERROR), 150 },
  { OV(1697 - SCALE_HARDWARE_ERROR), 155 },
  { OV(1730 - SCALE_HARDWARE_ERROR), 160 },
  { OV(1762 - SCALE_HARDWARE_ERROR), 165 },
  { OV(1795 - SCALE_HARDWARE_ERROR), 170 },
  { OV(1834 - SCALE_HARDWARE_ERROR), 176 },
  { OV(1853 - SCALE_HARDWARE_ERROR), 179 },
  { OV(1879 - SCALE_HARDWARE_ERROR), 183 },
  { OV(1905 - SCALE_HARDWARE_ERROR), 187 },
  { OV(1924 - SCALE_HARDWARE_ERROR), 190 },
  { OV(1956 - SCALE_HARDWARE_ERROR), 195 },
  { OV(1981 - SCALE_HARDWARE_ERROR), 199 },
  { OV(2013 - SCALE_HARDWARE_ERROR), 204 },
  { OV(2045 - SCALE_HARDWARE_ERROR), 209 },
  { OV(2083 - SCALE_HARDWARE_ERROR), 215 },
  { OV(2170 - SCALE_HARDWARE_ERROR), 229 },
  { OV(2120 - SCALE_HARDWARE_ERROR), 221 },
  { OV(2220 - SCALE_HARDWARE_ERROR), 237 },
  { OV(2282 - SCALE_HARDWARE_ERROR), 247 },
  { OV(2350 - SCALE_HARDWARE_ERROR), 258 },
  { OV(2435 - SCALE_HARDWARE_ERROR), 272 },
  { OV(2544 - SCALE_HARDWARE_ERROR), 290 },
  { OV(2603 - SCALE_HARDWARE_ERROR), 300 },
  { OV(2897 - SCALE_HARDWARE_ERROR), 350 }
};

// #undef OV_SCALE
// #define OV_SCALE(N) (N)
