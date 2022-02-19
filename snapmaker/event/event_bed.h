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
