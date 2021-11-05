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

constexpr temp_entry_t temptable_25[] PROGMEM = {
  { OV((628>>2)), -30 },
  { OV((628>>2)), -27 },
  { OV((628>>2)), -10 },
  { OV((628>>2)),   0 },
  { OV((693>>2)),   9 },
  { OV((751>>2)),  17 },
  { OV((794>>2)),  23 },
  { OV((829>>2)),  28 },
  { OV((907>>2)),  39 },
  { OV((950>>2)),  45 },
  { OV((992>>2)),  51 },
  { OV((1020>>2)),  55 },
  { OV((1034>>2)),  57 },
  { OV((1090>>2)),  65 },
  { OV((1117>>2)),  69 },
  { OV((1138>>2)),  72 },
  { OV((1187>>2)),  79 },
  { OV((1215>>2)),  85 },
  { OV((1262>>2)),  90 },
  { OV((1296>>2)),  95 },
  { OV((1330>>2)), 100 },
  { OV((1357>>2)), 105 },
  { OV((1405>>2)), 111 },
  { OV((1445>>2)), 117 },
  { OV((1485>>2)), 123 },
  { OV((1519>>2)), 128 },
  { OV((1552>>2)), 133 },
  { OV((1592>>2)), 139 },
  { OV((1625>>2)), 144 },
  { OV((1664>>2)), 150 },
  { OV((1697>>2)), 155 },
  { OV((1730>>2)), 160 },
  { OV((1762>>2)), 165 },
  { OV((1795>>2)), 170 },
  { OV((1834>>2)), 176 },
  { OV((1853>>2)), 179 },
  { OV((1879>>2)), 183 },
  { OV((1905>>2)), 187 },
  { OV((1924>>2)), 190 },
  { OV((1956>>2)), 195 },
  { OV((1981>>2)), 199 },
  { OV((2013>>2)), 204 },
  { OV((2045>>2)), 209 },
  { OV((2083>>2)), 215 },
  { OV((2170>>2)), 229 },
  { OV((2120>>2)), 221 },
  { OV((2220>>2)), 237 },
  { OV((2282>>2)), 247 },
  { OV((2350>>2)), 258 },
  { OV((2435>>2)), 272 },
  { OV((2544>>2)), 290 },
  { OV((2603>>2)), 300 },
  { OV((2897>>2)), 350 }
};

// #undef OV_SCALE
// #define OV_SCALE(N) (N)
