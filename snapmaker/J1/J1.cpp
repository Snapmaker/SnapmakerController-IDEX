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
#include "src/module/stepper.h"
#include "../../Marlin/src/module/temperature.h"
#include "../module/power_loss.h"
#include "../module/motion_control.h"
#include "../module/exception.h"
#include "../../../src/module/AxisManager.h"

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

void axis_speed_update() {
  static uint32 last_tick = 0;

  if (PENDING(millis(), last_tick + 10)) {
    return;
  }

  last_tick = millis();
  static float dump_speed = 0;
  dump_speed += axisManager.axis[0].getCurrentSpeedMMs();
  dump_speed += axisManager.axis[1].getCurrentSpeedMMs();
  dump_speed += axisManager.axis[2].getCurrentSpeedMMs();
}

static uint32_t z_move_count = 0;
uint16_t z_sg_value = 0;

static uint32_t y_move_count = 0;
uint16_t y_sg_value = 0;

static uint32_t x0_move_count = 0;
uint16_t x0_sg_value = 0;

static uint32_t x1_move_count = 0;
uint16_t x1_sg_value = 0;

void z_sg_value_set(void) {
  if (!z_sg_value) {
    if (axisManager.axis[2].cur_speed > 3) {
      z_move_count++;
      if (z_move_count > 50) {
        z_sg_value = (float)(stepperZ.SG_RESULT()) / 3 - 25;
        LOG_I("z_sg_value set to %d\r\n", z_sg_value);
        extern bool z_homing;
        extern bool z_stall_guard_setting;
        if (z_homing) {
          z_stall_guard_setting = true;
          uint8_t z_sg_value_set = z_sg_value * 1.5;
          motion_control.enable_stall_guard_only_axis(Z_AXIS, z_sg_value_set);
          motion_control.clear_trigger();
          LOG_I("Z home stall gurad set to %d\r\n", z_sg_value_set);
        }
      }
    }
    else {
      z_move_count = 0;
    }
  }
}

void y_sg_value_set(void) {
  if (!y_sg_value) {
    // XY calibration move for y: F9000 * 100 / 201
    if (fabs(axisManager.axis[1].cur_speed - ((MOTION_TRAVEL_FEADRATE) * 100 / 60 / 201)) < 10) {
      y_move_count++;
      if (y_move_count > 10) {
        uint32_t sg = stepperY.SG_RESULT();
        // if (sg > 120) {
        //   y_sg_value = 30 + (sg - 120) / 10;
        // }
        // else {
        //   y_sg_value = (float)sg / 4;
        // }
        y_sg_value = sg / 10;
        LOG_I("y_sg_value set to %d\r\n", y_sg_value);
      }
    }
    else {
      y_move_count = 0;
    }
  }
}

void x0_sg_value_set(void) {
  if (!x0_sg_value) {
    // XY calibration move for x: F9000 * 175 / 201
    if (fabs(axisManager.axis[0].cur_speed - ((MOTION_TRAVEL_FEADRATE) * 175 / 60 / 201)) < 10 && active_extruder == 0) {
      x0_move_count++;
      if (x0_move_count > 10) {
        uint32_t sg = stepperX.SG_RESULT();
        // if (sg > 120) {
        //   x0_sg_value = 30 + (sg - 120) / 10;
        // }
        // else {
        //   x0_sg_value = (float)sg / 4;
        // }
        x0_sg_value = sg / 12;
        LOG_I("x0_sg_value set to %d\r\n", x0_sg_value);
      }
    }
    else {
      x0_move_count = 0;
    }
  }
}

void x1_sg_value_set(void) {
  if (!x1_sg_value) {
    if (fabs(axisManager.axis[0].cur_speed - ((MOTION_TRAVEL_FEADRATE)/60)) < 10 && active_extruder == 1) {
      x1_move_count++;
      if (x1_move_count > 10) {
        uint32_t sg = stepperX2.SG_RESULT();
        // if (sg > 120) {
        //   x1_sg_value = 30 + (sg - 120) / 10;
        // }
        // else {
        //   x1_sg_value = (float)sg / 4;
        // }
        x1_sg_value = sg / 12;
        LOG_I("x1_sg_value set to %d\r\n", x1_sg_value);
      }
    }
    else {
      x1_move_count = 0;
    }
  }
}

void sg_set(void) {
  static uint32 last_tick = 0;

  if (PENDING(millis(), last_tick + 10)) {
    return;
  }
  z_sg_value_set();
  y_sg_value_set();
  x0_sg_value_set();
  x1_sg_value_set();
}

void probe_io_log(void) {
  static uint8_t last_probe_0 = 0;
  static uint8_t last_probe_1 = 0;

  uint8_t t = READ(X0_CAL_PIN);
  if (t != last_probe_0) {
    last_probe_0 = t;
    LOG_I("probe_0 %d\r\n", last_probe_0);
  }
  t = READ(X1_CAL_PIN);
  if (t != last_probe_1) {
    last_probe_1 = t;
    LOG_I("probe_1 %d\r\n", last_probe_1);
  }

}

void TMC2209_log(void) {
  static uint32_t last_tick = 0;
  if (PENDING(millis(), last_tick))
    return;

  last_tick = millis() + 100;

  static uint32_t last_TSTEP = 0;
  uint32_t t = stepperX.TSTEP();
  if (last_TSTEP != t) {
    last_TSTEP = t;
    LOG_I("last_TSTEP %d\r\n", last_TSTEP);
  }
}


void statistics_log(void) {

  extern uint32_t statistics_slowdown_cnt;
  extern uint32_t statistics_abort_cnt;
  extern uint32_t statistics_gcode_timeout_cnt;
  extern uint32_t statistics_no_step_but_has_block_cnt;
  extern uint32_t statistics_funcgen_runout_cnt;

  static uint32_t last_statistics_slowdown_cnt;
  static uint32_t last_statistics_abort_cnt;
  static uint32_t last_statistics_gcode_timeout_cnt;
  static uint32_t last_statistics_no_step_but_has_block_cnt;
  static uint32_t last_statistics_funcgen_runout_cnt;

  if (last_statistics_slowdown_cnt != statistics_slowdown_cnt) {
    LOG_I("statistics_slowdown_cnt %d\r\n", statistics_slowdown_cnt);
    last_statistics_slowdown_cnt = statistics_slowdown_cnt;
  }

  if (last_statistics_abort_cnt != statistics_abort_cnt) {
    LOG_I("statistics_abort_cnt %d\r\n", statistics_abort_cnt);
    last_statistics_abort_cnt = statistics_abort_cnt;
  }


  if (last_statistics_gcode_timeout_cnt != statistics_gcode_timeout_cnt) {
    LOG_I("statistics_gcode_timeout_cnt %d\r\n", statistics_gcode_timeout_cnt);
    last_statistics_gcode_timeout_cnt = statistics_gcode_timeout_cnt;
  }


  if (last_statistics_no_step_but_has_block_cnt != statistics_no_step_but_has_block_cnt) {
    LOG_I("statistics_no_step_but_has_block_cnt %d\r\n", statistics_no_step_but_has_block_cnt);
    last_statistics_no_step_but_has_block_cnt = statistics_no_step_but_has_block_cnt;
  }

  if (last_statistics_funcgen_runout_cnt != statistics_funcgen_runout_cnt) {
    LOG_I("statistics_funcgen_runout_cnt %d\r\n", statistics_funcgen_runout_cnt);
    last_statistics_funcgen_runout_cnt = statistics_funcgen_runout_cnt;
  }

}

#define ROUND_MILLIME  (52.1)
#define FILAMENT_SENSOR_ADC_TO_MM (ROUND_MILLIME / 4096)

void filament_sensor_log(uint8_t e0_e1) {
  static uint32_t log_tick = 0;
  static uint32_t last_tick = 0;
  static int32_t last_e_adc = -1;

  static float e_sum_millimeter = 0.0;
  static float e_aligne_millimeter = 0.0;
  static float e_this_round_millimeter = 0.0;

  static bool e_dead_aligned = false;
  static int32_t e_round_cnt = 0;

  float delta_millims;

  if (PENDING(millis(), last_tick))
    return;

  last_tick = millis() + 100;

  int32_t e_adc = 0;
  for (int i = 0; i < 5; i++) {
    e_adc += e0_e1 ? (4096 - analogRead(FILAMENT1_ADC_PIN)) : analogRead(FILAMENT0_ADC_PIN);
  }
  e_adc = e_adc / 5;

  if (-1 == last_e_adc) last_e_adc = e_adc;

  if (abs(e_adc - last_e_adc) < 2048) {
    delta_millims = (e_adc - last_e_adc) * FILAMENT_SENSOR_ADC_TO_MM;
    if (e_adc > last_e_adc)
      delta_millims = -fabs(delta_millims);
    else
      delta_millims = fabs(delta_millims);
    last_e_adc = e_adc;
  }
  else {
    delta_millims = (abs(e_adc - last_e_adc) & 4096) * FILAMENT_SENSOR_ADC_TO_MM;
    if (e_adc < last_e_adc && e_adc > 30) {
      if (!e_dead_aligned)
        e_round_cnt--;
      delta_millims = -fabs(delta_millims);
      last_e_adc = e_adc;
    }
    else if (e_adc > last_e_adc && e_adc < 4066){
      if (e_dead_aligned) {
        if (e_this_round_millimeter > 0.7 * FILAMENT_SENSOR_ADC_TO_MM) {
          e_this_round_millimeter = 0;
          e_round_cnt++;
        }
      }
      else {
        e_round_cnt++;
      }

      delta_millims = fabs(delta_millims);
      last_e_adc = e_adc;
    }
    else {
      delta_millims = 0.0;
    }
  }

  e_this_round_millimeter += delta_millims;

  if (!e_dead_aligned) {
    if (0 != e_round_cnt) {
      e_dead_aligned = true;
      e_round_cnt = 0;
      e_aligne_millimeter = e_this_round_millimeter;
      e_this_round_millimeter = 0;
    }
  }
  e_sum_millimeter = e_aligne_millimeter + e_round_cnt * ROUND_MILLIME + e_this_round_millimeter;

  log_tick++;
  if (log_tick > 10) {
    LOG_I("E%d: ADC %d, round %d, millis %f, \r\n", e0_e1, e_adc, e_round_cnt, e_sum_millimeter);
    log_tick = 0;
  }

}


void j1_main_task(void *args) {
  uint32_t syslog_timeout = millis();

  log_reset_source();

  while(1) {
    print_control.loop();
    // power_loss.process();
    // printer_event_loop();
    // exception_event_loop();
    // local_event_loop();

    axis_speed_update();
    sg_set();
    probe_io_log();
    // TMC2209_log();
    statistics_log();
    // filament_sensor_log(0);
    // filament_sensor_log(1);

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
      // LOG_E("Starve dog for %d ms\r\n", starve_dog_time_ms);
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
