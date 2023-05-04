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

#ifndef EVENT_BED_H
#define EVENT_BED_H
#include "event_base.h"


enum {
  BED_ID_REPORT_INFO             = 0x01,
  BED_ID_SET_TEMPERATURE         = 0x02,
  BED_ID_REPORT_TEMPERATURE      = 0xA0,
};

#define BED_ID_CB_COUNT 3

extern event_cb_info_t bed_cb_info[BED_ID_CB_COUNT];
#endif
