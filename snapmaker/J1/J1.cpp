#include "J1.h"
#include "../event/event.h"
#include "../event/subscribe.h"
#include "../protocol/protocol_sacp.h"
#include "switch_detect.h"
#include "../module/update.h"
#include "../module/fdm.h"

void J1_setup() {
  update_server.init();
  switch_detect.init();
  fdm_head.init();
  debug.init();
  subscribe_init();
  event_init();
}
