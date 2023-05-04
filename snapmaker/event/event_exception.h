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

#ifndef EVENT_EXCEPTION_H
#define EVENT_EXCEPTION_H
#include "event_base.h"

enum {
  EXCEPTION_ID_TIRGGER_REPORT  = 0x00,  // J1 report to SC or PC
  EXCEPTION_ID_CLEAN_REPORT    = 0x01,  // J1 report to SC or PC
  EXCEPTION_ID_GET             = 0x02,  // SC or PC get exception info
  EXCEPTION_ID_SC_CLEAN        = 0x03,  // SC or PC clean exception info
};
#define EXCEPTION_ID_CB_COUNT 4
extern event_cb_info_t exception_cb_info[EXCEPTION_ID_CB_COUNT];
void exception_event_loop(void);
#endif
