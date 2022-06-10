#include "event_bed.h"
#include "../module/bed_control.h"

static ErrCode bed_report_info(event_param_t& event) {
  bed_control_info_t * info = (bed_control_info_t *)(event.data + 1);
  event.data[0] = E_SUCCESS;
  bed_control.get_info(*info);
  LOG_V("SC req bed info, temp: %.2f/%d\n", INT_TO_FLOAT(info->cur_temp), info->target_temp);
  event.length = sizeof(bed_control_info_t) + 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode bed_set_temperature(event_param_t& event) {
  int16_t target = *((int16_t *)(event.data + 2));
  LOG_V("SC set bed temp to: %d\n", target);
  event.data[0] = bed_control.set_temperature(target);;
  event.length = 1;
  return send_event(event);
}

event_cb_info_t bed_cb_info[BED_ID_CB_COUNT] = {
  {BED_ID_REPORT_INFO             , EVENT_CB_DIRECT_RUN, bed_report_info},
  {BED_ID_SET_TEMPERATURE         , EVENT_CB_DIRECT_RUN, bed_set_temperature},
  {BED_ID_REPORT_TEMPERATURE      , EVENT_CB_DIRECT_RUN, bed_report_info},
};
