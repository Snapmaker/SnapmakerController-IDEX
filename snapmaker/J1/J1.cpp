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

TaskHandle_t thandle_event_loop = NULL;
TaskHandle_t thandle_event_recv = NULL;
TaskHandle_t thandle_j1_main = NULL;
TaskHandle_t thandle_subscribe = NULL;

uint32_t feed_dog_time = 0;
uint32_t max_starve_dog_time = 0;

void log_reset_source(void) {
  extern unsigned int ahbrst_reg;
  LOG_I("Reset source: ");
  if (ahbrst_reg & (1<<31)) {
    LOG_I(" low power\r\n");
  }
  else if (ahbrst_reg & (1<<30)) {
    LOG_I(" window watchDog\r\n");
  }
  else if (ahbrst_reg & (1<<29)) {
    LOG_I(" indepenten watchDog\r\n");
  }
  else if (ahbrst_reg & (1<<28)) {
    LOG_I(" software\r\n");
  }
  else if (ahbrst_reg & (1<<27)) {
    LOG_I(" power on\r\n");
  }
  else if (ahbrst_reg & (1<<26)) {
    LOG_I(" extern reset pin\r\n");
  }
  else {
    LOG_I("unknown\r\n");
  }
}

void j1_main_task(void *args) {
  uint32_t syslog_timeout = millis();

  log_reset_source();

  while(1) {
    print_control.loop();
    // power_loss.process();

    if (ELAPSED(millis(), syslog_timeout)) {
      syslog_timeout = millis() + 20000;
      LOG_I("c0: %d/t0: %d, c1: %d/t1: %d, cb: %d/tb: %d, ",
        (int)thermalManager.degHotend(0), thermalManager.degTargetHotend(0),
        (int)thermalManager.degHotend(1), thermalManager.degTargetHotend(1),
        (int)thermalManager.degBed(), thermalManager.degTargetBed());
      LOG_I("sta: %u, excep sta: 0x%x, excep beh: 0x%x\n", system_service.get_status(),
        exception_server.get_exception(), exception_server.get_behavior());
    }

    uint32_t starve_dog_time_ms = (uint32_t)(millis() - feed_dog_time);
    if (ELAPSED(millis(), feed_dog_time + 1000)) {
      LOG_E("Starve dog for %d ms\r\n", starve_dog_time_ms);
    }

    if (max_starve_dog_time < starve_dog_time_ms) {
      max_starve_dog_time = starve_dog_time_ms;
      //LOG_E("max_starve_dog_time = %d \r\n", max_starve_dog_time);
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

  TaskHandle_t thandle_j1_main = NULL;
  BaseType_t ret = xTaskCreate(j1_main_task, "j1_main_task", 1024, NULL, 5, &thandle_j1_main);
  if (ret != pdPASS) {
    SERIAL_ECHO("Failed to create j1_main_task!\n");
  }
  else {
    SERIAL_ECHO("Created j1_main_task task!\n");
  }
}
