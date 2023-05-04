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

#ifndef EVENT_ENCLOUSER_H
#define EVENT_ENCLOUSER_H
#include "event_base.h"

enum {
  ENCLOUSER_ID_REPORT_INFO       = 0x01,
  ENCLOUSER_ID_SET_LIGHT         = 0x02,
  ENCLOUSER_ID_SET_FAN           = 0x04,
  ENCLOUSER_ID_SUBSCRIBE_INFO    = 0xA1,
};

#define ENCLOUSER_ID_CB_COUNT  4
extern event_cb_info_t enclouser_cb_info[ENCLOUSER_ID_CB_COUNT];

#endif
