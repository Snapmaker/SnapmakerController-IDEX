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

#ifndef ENCLOSURE_H
#define ENCLOSURE_H
#include "../J1/common_type.h"
#include "module_base.h"

typedef enum {
  ENCLOSURE_FAN_OFF,
  ENCLOSURE_FAN_ON,
} enclosure_fan_status_e;

class Enclosure {
  public:
    void get_module_info(module_info_t &info);
    uint8_t get_key();
    uint8_t get_light_power();
    void set_light_power(uint8_t power);
    uint8_t get_fan_power();
    void set_fan_power(uint8_t power);
  private:
    enclosure_fan_status_e fan_status = ENCLOSURE_FAN_OFF;
};

extern Enclosure enclosure;

#endif
