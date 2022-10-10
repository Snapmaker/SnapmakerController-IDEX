#include "J1.h"
#include "../event/event.h"
#include "../event/subscribe.h"
#include "../protocol/protocol_sacp.h"
#include "switch_detect.h"
#include "../module/update.h"
#include "../module/fdm.h"
#include "../event/event_printer.h"
#include "../event/event_exception.h"
#include "../module/print_control.h"
#include "../../Marlin/src/module/temperature.h"
#include "../module/power_loss.h"
#include "../module/exception.h"


void j1_main_task(void *args) {
  uint32_t syslog_timeout = millis();

  while(1) {
    print_control.loop();
    power_loss.process();

    if (ELAPSED(millis(), syslog_timeout)) {
      syslog_timeout = millis() + 20000;
      LOG_I("c0: %d/t0: %d, c1: %d/t1: %d, cb: %d/tb: %d, ",
        (int)thermalManager.degHotend(0), thermalManager.degTargetHotend(0),
        (int)thermalManager.degHotend(1), thermalManager.degTargetHotend(1),
        (int)thermalManager.degBed(), thermalManager.degTargetBed());
      LOG_I("sta: %u, excep sta: 0x%x, excep beh: 0x%x\n", system_service.get_status(),
        exception_server.get_exception(), exception_server.get_behavior());
    }

    watchdog_refresh();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void J1_setup() {
  update_server.init();
  switch_detect.init();
  fdm_head.init();
  debug.init();
  subscribe_init();
  event_init();

  BaseType_t ret = xTaskCreate(j1_main_task, "j1_main_task", 1024, NULL, 5, NULL);
  if (ret != pdPASS) {
    SERIAL_ECHO("Failed to create j1_main_task!\n");
  }
  else {
    SERIAL_ECHO("Created j1_main_task task!\n");
  }
}
