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

#ifndef EVEVT_UPDATE_H
#define EVEVT_UPDATE_H
#include "../J1/common_type.h"
#include "event_base.h"

enum : uint8_t {
  UPDATE_ID_REQ_UPDATE              = 0x01,
  UPDATE_ID_REQ_UPDATE_PACK         = 0x02,
  UPDATE_ID_REPORT_STATUS           = 0x03,
};

#define UPDATE_ID_CB_COUNT 1
extern event_cb_info_t update_cb_info[UPDATE_ID_CB_COUNT];

#endif
