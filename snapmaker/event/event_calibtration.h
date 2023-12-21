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

#ifndef EVENT_CAlIBRATION_H
#define EVENT_CAlIBRATION_H
#include "event_base.h"

enum {
  CAlIBRATION_ID_SET_MODE            = 0x00,
  CAlIBRATION_ID_MOVE_TO_POSITION    = 0x01,
  CAlIBRATION_ID_START_BED_PROBE     = 0x02,
  CAlIBRATION_ID_EXIT                = 0x06,
  CAlIBRATION_ID_REPORT_STATUS       = 0x07,
  CAlIBRATION_ID_RETRACK_E           = 0x08,
  CAlIBRATION_ID_REPORT_BED_OFFSET   = 0xA0,
  CAlIBRATION_ID_MOVE_NOZZLE         = 0x11,
  CAlIBRATION_ID_SET_Z_OFFSET        = 0x15,
  CAlIBRATION_ID_GET_Z_OFFSET        = 0x16,
  CAlIBRATION_ID_START_XY            = 0x21,
  CAlIBRATION_ID_SET_XY_OFFSET       = 0x22,
  CAlIBRATION_ID_REPORT_XY_OFFSET    = 0x23,
  CAlIBRATION_ID_START_PID_AUTOTUNE  = 0x40,
  CAlIBRATION_ID_SUBSCRIBE_Z_OFFSET    = 0xA2,
};

#define CAlIBRATION_ID_CB_COUNT 14

extern event_cb_info_t calibtration_cb_info[CAlIBRATION_ID_CB_COUNT];
#endif
