/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
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

#include "debug.h"
#include "../event/event_base.h"
#include "../event/event_system.h"

#if (SNAP_DEBUG == 1)

SnapDebug debug;

#if defined (__GNUC__)                /* GNU GCC Compiler */
  /* the version of GNU GCC must be greater than 4.x */
  typedef __builtin_va_list       __gnuc_va_list;
  typedef __gnuc_va_list          va_list;
  #define va_start(v,l)           __builtin_va_start(v,l)
  #define va_end(v)               __builtin_va_end(v)
  #define va_arg(v,l)             __builtin_va_arg(v,l)
#else
  #error "Snap debug only support GNU compiler for now"
#endif

static debug_level_e  debug_msg_level = SNAP_DEBUG_LEVEL_VERBOSE;
static char log_buf[SNAP_LOG_BUFFER_SIZE + 2];

const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_TRACE_STR,
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};

// output debug message, will not output message whose level
// is less than msg_level
// param:
//    level - message level
//    fmt - format of messages
//    ... - args
void SnapDebug::Log(debug_level_e level, const char *fmt, ...) {
  va_list args;

  if (level < debug_msg_level)
    return;

  va_start(args, fmt);
  char * data_data = log_buf + 2;
  vsnprintf(data_data, SNAP_LOG_BUFFER_SIZE - 2, fmt, args);
  log_buf[0] = E_SUCCESS;
  log_buf[1] = level;
  va_end(args);

  SACP_head_base_t sacp = {SACP_ID_HMI, SACP_ATTR_ACK, 0, COMMAND_SET_SYS, SYS_ID_REPORT_LOG};
  send_event(EVENT_SOURCE_HMI, sacp, (uint8_t*)log_buf, strlen(data_data) + 2);
  if (!evevnt_serial[EVENT_SOURCE_MARLIN]->enable_sacp()) {
    SERIAL_ECHO(data_data);
  } else {
    sacp.recever_id = SACP_ID_PC;
    send_event(EVENT_SOURCE_MARLIN, sacp, (uint8_t*)log_buf, strlen(data_data) + 2);
  }
}


// set current debug level message level less than this level
// will not be outputed, set by M2000
void SnapDebug::set_level(debug_level_e l) {
  Log(SNAP_DEBUG_LEVEL_INFO, "old debug level: %d\n", debug_msg_level);

  if (l > SNAP_DEBUG_LEVEL_MAX)
    return;
 
  debug_msg_level = l;
}

debug_level_e SnapDebug::get_level() {
  return debug_msg_level;
}

#endif // #if (SNAP_DEBUG == 1)
