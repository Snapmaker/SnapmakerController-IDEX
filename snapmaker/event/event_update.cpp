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

#include "event_update.h"
#include "../module/update.h"


static ErrCode req_start_update(event_param_t& event) {
  update_packet_info_t * head = (update_packet_info_t *)(event.data+2);

  if (event.length < sizeof(update_packet_info_t)) {
    SERIAL_ECHOLNPAIR("update pack head len failed");
    event.data[0] = 1;
    event.length = 1;
    return send_event(event);
  }
  event.data[0] = update_server.is_allow_update(head);
  event.length = 1;
  ErrCode ret = send_event(event);

  if (event.data[0] != E_SUCCESS) {
    SERIAL_ECHOLNPAIR("update pack head parse failed");
  } else {
    update_server.save_update_info(head, event.source, event.info.recever_id);
    update_server.just_to_boot();
  }
  return ret;
}

event_cb_info_t update_cb_info[UPDATE_ID_CB_COUNT] = {
  {UPDATE_ID_REQ_UPDATE      , EVENT_CB_DIRECT_RUN, req_start_update}
};