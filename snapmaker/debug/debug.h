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

#ifndef SNAPMAKER_DEBUG_H_
#define SNAPMAKER_DEBUG_H_

#include <stdio.h>
#include "MapleFreeRTOS1030.h"

// 1 = enable API for snap debug
#define SNAP_DEBUG 1

enum debug_level_e : uint8_t {
  SNAP_DEBUG_LEVEL_TRACE = 0,
  SNAP_DEBUG_LEVEL_VERBOSE,
  SNAP_DEBUG_LEVEL_INFO,
  SNAP_DEBUG_LEVEL_WARNING,
  SNAP_DEBUG_LEVEL_ERROR,
  SNAP_DEBUG_LEVEL_FATAL,
  SNAP_DEBUG_LEVEL_MAX
};

#define SNAP_DEBUG_LEVEL_DEFAULT SNAP_DEBUG_LEVEL_INFO

// state for Gcode command
enum GcodeState : uint8_t {
  GCODE_STATE_RECEIVED,
  GCODE_STATE_CHK_ERR,
  GCODE_STATE_BUFFERED,
  GCODE_STATE_ACKED,
  GCODE_STATE_INVALID
};

#if (SNAP_DEBUG)

// massage will output to this interface
#define CONSOLE_OUTPUT(log) MYSERIAL0.print(log)

// log buffer size, max length for one debug massage
#define SNAP_LOG_BUFFER_SIZE 256

#define SNAP_TRACE_STR    "TRACE"
#define SNAP_VERBOS_STR   "VERBOS"
#define SNAP_INFO_STR     "INFO"
#define SNAP_WARNING_STR  "WARN"
#define SNAP_ERROR_STR    "ERR"
#define SNAP_FATAL_STR    "FATAL"
extern const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX];
class SnapDebug {
  public:
    void init();
    void Log(debug_level_e level, const char *fmt, ...);
    void set_level(debug_level_e l);
    debug_level_e get_level();
    void show_all_status();

  private:
    SemaphoreHandle_t lock = NULL;
};

// interface for external use
// when SNAP_DEBUG is not defined, API is NONE

extern SnapDebug debug;

#define LOG_F(...) debug.Log(SNAP_DEBUG_LEVEL_FATAL, __VA_ARGS__)
#define LOG_E(...) debug.Log(SNAP_DEBUG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_W(...) debug.Log(SNAP_DEBUG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_I(...) debug.Log(SNAP_DEBUG_LEVEL_INFO, __VA_ARGS__)
#define LOG_V(...) debug.Log(SNAP_DEBUG_LEVEL_VERBOSE, __VA_ARGS__)
#define LOG_T(...) debug.Log(SNAP_DEBUG_LEVEL_TRACE, __VA_ARGS__)

#define SNAP_DEBUG_SET_LEVEL(l)        debug.set_level((debug_level_e)(l));
#define SNAP_DEBUG_IF_LEVEL(l)        (debug.get_level() <= (debug_level_e)(l))

#else

#define LOG_F(...)
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_V(...)
#define LOG_T(...)

#define SNAP_DEBUG_SHOW_INFO()
#define SNAP_DEBUG_SHOW_EXCEPTION()
#define SNAP_DEBUG_SET_LEVEL(l)
#define SNAP_DEBUG_CMD_CHECKSUM_ERROR(s)
#define SNAP_DEBUG_SET_GCODE_LINE(l)

#endif // #ifdef SNAP_DEBUG

#endif  // #ifndef SNAPMAKER_DEBUG_H_
