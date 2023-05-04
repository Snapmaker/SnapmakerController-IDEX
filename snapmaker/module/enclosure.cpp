/*
 * Snapmaker 3D Printer Firmware
 * Copyright (C) 2023 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of SnapmakerController-IDEX
 * (see https://github.com/Snapmaker/SnapmakerController-IDEX)
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "enclosure.h"
#include "../../Marlin/src/feature/caselight.h"
#include "../../Marlin/src/module/settings.h"

Enclosure enclosure;

void Enclosure::get_module_info(module_info_t &info) {
  info.key = get_key();
  info.module_id = MODULE_ENCLOSURE;
  info.module_index = 0;
  info.module_state = 0;
  info.sn = 0;
  info.version_length = MODULE_VER_LEN;
  info.version[0] = 0;
}

uint8_t Enclosure::get_key() {
  return MODULE_KEY(MODULE_ENCLOSURE, 0);
}

uint8_t Enclosure::get_light_power() {
  return caselight.get_power();
}

void Enclosure::set_light_power(uint8_t power) {
  caselight.set_power(power);
}

uint8_t Enclosure::get_fan_power() {
  return fan_status;
}

void Enclosure::set_fan_power(uint8_t power) {
}
