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
  event.data[0] = bed_control.set_temperature(target, false);
  event.length = 1;
  return send_event(event);
}

event_cb_info_t bed_cb_info[BED_ID_CB_COUNT] = {
  {BED_ID_REPORT_INFO             , EVENT_CB_DIRECT_RUN, bed_report_info},
  {BED_ID_SET_TEMPERATURE         , EVENT_CB_DIRECT_RUN, bed_set_temperature},
  {BED_ID_REPORT_TEMPERATURE      , EVENT_CB_DIRECT_RUN, bed_report_info},
};
