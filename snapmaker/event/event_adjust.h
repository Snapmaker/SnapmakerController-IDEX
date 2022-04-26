#ifndef EVENT_ADJUST_H
#define EVENT_ADJUST_H
#include "event_base.h"

enum {
  ADJUST_ID_SET_MODE            = 0x00,
  ADJUST_ID_MOVE_TO_POSITION    = 0x01,
  ADJUST_ID_START_BED_PROBE     = 0x02,
  ADJUST_ID_EXIT                = 0x06,
  ADJUST_ID_REPORT_STATUS       = 0x07,
  ADJUST_ID_RETRACK_E           = 0x08,
  ADJUST_ID_REPORT_BED_OFFSET   = 0xA0,
  ADJUST_ID_MOVE_NOZZLE         = 0x11,
  ADJUST_ID_SET_Z_OFFSET        = 0x15,
  ADJUST_ID_GET_Z_OFFSET        = 0x16,
  ADJUST_ID_START_XY            = 0x21,
  ADJUST_ID_SET_XY_OFFSET       = 0x22,
  ADJUST_ID_REPORT_XY_OFFSET    = 0x23,
};

#define ADJUST_ID_CB_COUNT 12

extern event_cb_info_t adjust_cb_info[ADJUST_ID_CB_COUNT];
#endif
