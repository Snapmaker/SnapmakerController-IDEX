#ifndef EVENT_EXCEPTION_H
#define EVENT_EXCEPTION_H
#include "event_base.h"

enum {
  EXCEPTION_ID_TIRGGER_REPORT  = 0x00,  // J1 report to SC or PC
  EXCEPTION_ID_CLEAN_REPORT    = 0x01,  // J1 report to SC or PC
  EXCEPTION_ID_GET             = 0x02,  // SC or PC get exception info
};
#define EXCEPTION_ID_CB_COUNT 3
extern event_cb_info_t exception_cb_info[EXCEPTION_ID_CB_COUNT];
void exception_event_loop(void);
#endif
