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
