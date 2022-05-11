#ifndef EVENT_ENCLOUSER_H
#define EVENT_ENCLOUSER_H
#include "event_base.h"

enum {
  ENCLOUSER_ID_REPORT_INFO       = 0x01,
  ENCLOUSER_ID_SET_LIGHT         = 0x02,
  ENCLOUSER_ID_SET_FAN           = 0x04,
  ENCLOUSER_ID_SUBSCRIBE_INFO    = 0xA0,
};

#define ENCLOUSER_ID_CB_COUNT  4
extern event_cb_info_t enclouser_cb_info[ENCLOUSER_ID_CB_COUNT];

#endif
