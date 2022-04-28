#include "J1.h"
#include "../event/event.h"
#include "../event/subscribe.h"
#include "../protocol/protocol_sacp.h"
#include "switch_detect.h"

void J1_setup() {
  switch_detect.init();
  subscribe_init();
  event_init();
}
