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

#ifndef EVENT_PRINTER_H
#define EVENT_PRINTER_H
#include "event_base.h"

enum {
  PRINTER_ID_REQ_FILE_INFO        = 0x00,
  PRINTER_ID_REPORT_STATUS        = 0x01,
  PRINTER_ID_REQ_GCODE            = 0x02,
  PRINTER_ID_START_WORK           = 0x03,
  PRINTER_ID_PAUSE_WORK           = 0x04,
  PRINTER_ID_RESUME_WORK          = 0x05,
  PRINTER_ID_STOP_WORK            = 0x06,
  PRINTER_ID_REQ_PL_STATUS        = 0x07,  // PL -> power loss
  PRINTER_ID_PL_RESUME            = 0x08,  // power loss resume
  PRINTER_ID_CLEAN_PL_DATA        = 0x09,
  PRINTER_ID_SET_MODE             = 0x0A,
  PRINTER_ID_REQ_AUTO_PARK_STATUS = 0x0B,
  PRINTER_ID_SET_PRINT_OFFSET     = 0x0C,
  PRINTER_ID_STOP_SINGLE_EXTRUDE  = 0x0D,
  PRINTER_ID_SET_WORK_PERCENTAGE  = 0x0E,
  PRINTER_ID_GET_WORK_PERCENTAGE  = 0x0F,
  PRINTER_ID_SET_FLOW_PERCENTAGE  = 0x10,
  PRINTER_ID_GET_FLOW_PERCENTAGE  = 0x11,
  PRINTER_ID_SET_TEMPERATURE_LOCK = 0x12,
  PRINTER_ID_GET_TEMPERATURE_LOCK = 0x13,
  PRINTER_ID_GET_FDM_ENABLE       = 0x19,
  PRINTER_ID_SET_NOISE_MODE       = 0x1c,
  PRINTER_ID_GET_NOISE_MODE       = 0x1d,
  PRINTER_ID_REQ_LINE             = 0xA0,
  PRINTER_ID_SUBSCRIBE_PRINT_MODE = 0xA1,
  PRINTER_ID_GET_WORK_FEEDRATE    = 0xA2,
  PRINTER_ID_SUBSCRIBE_FLOW_PERCENTAGE  = 0xA3,
  PRINTER_ID_SUBSCRIBE_WORK_PERCENTAGE  = 0xA4,
  PRINTER_ID_SUBSCRIBE_WORK_TIME        = 0xA5,
};

#define PRINTER_ID_CB_COUNT 28

extern event_cb_info_t printer_cb_info[PRINTER_ID_CB_COUNT];
void printer_event_init(void);
void printer_event_loop(void);
#endif
