#ifndef EVENT_FDM_H
#define EVENT_FDM_H
#include "event_base.h"

enum : uint8_t {
  FDM_ID_GET_INFO                = 0x01,
  FDM_ID_SET_TEMPERATURE         = 0x02,
  FDM_ID_SET_WORK_SPEED          = 0x03,
  FDM_ID_ENABLE_FILAMENT_SENSOR  = 0x04,
  FDM_ID_CHANGE_TOOL_HEAD        = 0x05,
  FDM_ID_SET_FAN_SPEED           = 0x06,
  FDM_ID_SET_NOZZLE_SPACING      = 0x07,
  FDM_ID_GET_NOZZLE_SPACING      = 0x08,
  FDM_ID_EXTRUSION_CONTROL       = 0x09,
  FDM_ID_EXTRUSION_UNTIL         = 0x0A,
  FDM_ID_REPORT_SUBSCRIBE_INFO   = 0xA0,
};

#define FDM_ID_CB_COUNT 11

extern event_cb_info_t fdm_cb_info[FDM_ID_CB_COUNT];


#endif
