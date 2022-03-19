#include "event_base.h"
#include "event_enclouser.h"
#include "../module/enclosure.h"


#pragma pack(1)

typedef struct {
  uint8 result;
  uint8_t key;
  uint8 module_status;  // Fixed 2, normal
  uint8_t light_power;
  bool  is_door_enabled;  // J1 not support
  bool  is_door_open;  // J1 not support
  uint8 fan_power;
} enclouser_info_t;

#pragma pack()

static ErrCode enclouser_report_info(event_param_t& event) {
  enclouser_info_t * info = (enclouser_info_t *)event.data;
  info->result = E_SUCCESS;
  info->key = enclosure.get_key();

  return send_event(event);
}

static ErrCode enclouser_set_light(event_param_t& event) {
  uint8_t power = event.data[1];
  SERIAL_ECHOLNPAIR("SC set enclouser light power:", power);
  enclosure.set_light_power(power);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode enclouser_set_fan(event_param_t& event) {
  uint8_t power = event.data[1];  // 0 - 100
  SERIAL_ECHOLNPAIR("SC set enclouser fan power:", power);
  enclosure.set_fan_power(power);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

event_cb_info_t enclouser_cb_info[ENCLOUSER_ID_CB_COUNT] = {
  {ENCLOUSER_ID_REPORT_INFO   , EVENT_CB_DIRECT_RUN, enclouser_report_info},
  {ENCLOUSER_ID_SET_LIGHT     , EVENT_CB_DIRECT_RUN, enclouser_set_light},
  {ENCLOUSER_ID_SET_FAN       , EVENT_CB_DIRECT_RUN, enclouser_set_fan},
  {ENCLOUSER_ID_SUBSCRIBE_INFO, EVENT_CB_DIRECT_RUN, enclouser_report_info},
};