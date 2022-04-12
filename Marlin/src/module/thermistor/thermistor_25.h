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
  { OV(822  - SCALE_HARDWARE_ERROR),  28 },
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
  { OV(1833 - SCALE_HARDWARE_ERROR), 179 },
  { OV(1858 - SCALE_HARDWARE_ERROR), 183 },
  { OV(1885 - SCALE_HARDWARE_ERROR), 187 },
  { OV(1904 - SCALE_HARDWARE_ERROR), 190 },
  { OV(1934 - SCALE_HARDWARE_ERROR), 195 },
  { OV(1961 - SCALE_HARDWARE_ERROR), 199 },
  { OV(1991 - SCALE_HARDWARE_ERROR), 204 },
  { OV(2023 - SCALE_HARDWARE_ERROR), 209 },
  { OV(2058 - SCALE_HARDWARE_ERROR), 215 },
  { OV(2089 - SCALE_HARDWARE_ERROR), 220 },
  { OV(2113 - SCALE_HARDWARE_ERROR), 225 },
  { OV(2144 - SCALE_HARDWARE_ERROR), 230 },
  { OV(2181 - SCALE_HARDWARE_ERROR), 235 },
  { OV(2211 - SCALE_HARDWARE_ERROR), 240 },
  { OV(2242 - SCALE_HARDWARE_ERROR), 245 },
  { OV(2266 - SCALE_HARDWARE_ERROR), 250 },
  { OV(2296 - SCALE_HARDWARE_ERROR), 255 },
  { OV(2333 - SCALE_HARDWARE_ERROR), 260 },
  { OV(2366 - SCALE_HARDWARE_ERROR), 265 },
  { OV(2396 - SCALE_HARDWARE_ERROR), 270 },
  { OV(2424 - SCALE_HARDWARE_ERROR), 275 },
  { OV(2453 - SCALE_HARDWARE_ERROR), 280 },
  { OV(2479 - SCALE_HARDWARE_ERROR), 285 },
  { OV(2508 - SCALE_HARDWARE_ERROR), 290 },
  { OV(2535 - SCALE_HARDWARE_ERROR), 295 },
  { OV(2566 - SCALE_HARDWARE_ERROR), 300 },
  { OV(2595 - SCALE_HARDWARE_ERROR), 305 },
  { OV(2624 - SCALE_HARDWARE_ERROR), 310 },
  { OV(2856 - SCALE_HARDWARE_ERROR), 350 }
};

// #undef OV_SCALE
// #define OV_SCALE(N) (N)
