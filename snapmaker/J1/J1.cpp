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
#include "../../Marlin/src/module/settings.h"
#include "../module/power_loss.h"
#include "../module/motion_control.h"
#include "../module/exception.h"
#include "../../../src/module/AxisManager.h"
#include "../module/factory_data.h"
#include "../module/calibtration.h"


TaskHandle_t thandle_event_loop = NULL;
TaskHandle_t thandle_event_recv = NULL;
TaskHandle_t thandle_j1_main = NULL;
TaskHandle_t thandle_subscribe = NULL;

uint32_t feed_dog_time = 0;
uint32_t max_starve_dog_time = 0;
bool ml_setting_need_save = false;

bool got_stepper_debug_info = false;
xyze_pos_t stepper_cur_position;
xyze_pos_t motion_cur_position;
xyze_pos_t motion_get_position;

float diff_x = 0.0f;
float diff_y = 0.0f;

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
        if (z_homing && print_control.get_z_home_sg()) {
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

// void probe_io_log(void) {
//   static uint8_t last_probe_0 = 0;
//   static uint8_t last_probe_1 = 0;

//   uint8_t t = READ(X0_CAL_PIN);
//   if (t != last_probe_0) {
//     last_probe_0 = t;
//     LOG_I("probe_0 %d\r\n", last_probe_0);
//   }
//   t = READ(X1_CAL_PIN);
//   if (t != last_probe_1) {
//     last_probe_1 = t;
//     LOG_I("probe_1 %d\r\n", last_probe_1);
//   }
// }

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

#if ENABLED(DEBUG_ISR_CPU_USAGE)
void step_isr_usage_log(void) {
  // char log_str[] = "ISR usage: %.2f%\n";
  float isr_usage;
  static uint32_t last_log_tick = 0;
  static uint16_t max_stepper_isr_delay = 0;
  static float max_stepper_isr_usage = 0.0;
  static bool need_log = false;

  isr_usage = (axisManager.counts[19] * 100.0 / STEPPER_TIMER_RATE);
  if (isr_usage > max_stepper_isr_usage) {
    max_stepper_isr_usage = isr_usage;
    need_log = true;
  }

  if (axisManager.counts[18] > max_stepper_isr_delay) {
    max_stepper_isr_delay = axisManager.counts[18];
    need_log = true;
  }

  // LOG_I(log_str, isr_usage);
  // if (isr_usage >= 40)
  //   LOG_I(log_str, isr_usage);
  if (PENDING(millis(), last_log_tick + 1000))
    return;

  axisManager.counts[19] = 0;
  last_log_tick = millis();

  if (need_log) {
    LOG_V("### max_stepper_isr_usage %.2f%\r\n", max_stepper_isr_usage);
    LOG_V("### max_stepper_isr_delay %d us\r\n", max_stepper_isr_delay / STEPPER_TIMER_TICKS_PER_US);
    need_log = false;
  }
}
#endif

#if 0
void float_round_test(float diff, uint32_t log_cnt) {

  uint32_t lc = 0;
  LOG_I("LROUND(0.1) = %d, LROUND(0.5) = %d, LROUND(-0.1) = %d, LROUND(-0.5) = %d\n", LROUND(0.1), LROUND(0.5), LROUND(-0.1), LROUND(-0.5));

  for(int i = 0; i < 200 * 80; i++) {
    float origin_float_y0 = 0.0f + i * 0.0001;
    float origin_float_y1 = origin_float_y0 + diff;

    int origin_int_y0 = LROUND(origin_float_y0 * 80);
    int origin_int_y1 = LROUND(origin_float_y1 * 80);

    // int origin_y0_y1_diff_int = abs(origin_int_y0 - origin_int_y1);

    for(int run_step = 0; run_step < 100000; run_step++) {
      float run_distanc = run_step * 0.001;

      float after_run_float_y0 = origin_float_y0 + run_distanc;
      float after_run_float_y1 = origin_float_y1 + run_distanc;

      int after_run_int_y0 = LROUND(after_run_float_y0 * 80);
      int after_run_int_y1 = LROUND(after_run_float_y1 * 80);

      int T1_actual_move_steps = after_run_int_y1 - origin_int_y1;
      int T0_actual_move_steps = origin_int_y0 - after_run_int_y0;
      int T0_after_move_step_pos = origin_int_y0 + T1_actual_move_steps + T0_actual_move_steps;

      // if (abs(origin_y0_y1_diff_int) != abs(after_run_y0_y1_diff_int)) {
      {
        lc++;
        LOG_I("\r\n\r\n====================, T0 T1 y axis offset: %f\r\n", diff);
        LOG_I("1) T0 working, before tool change");
        LOG_I("float coordinates: T0_float_y: %f, T1_float_y: %f:\n", origin_float_y0, origin_float_y1);

        LOG_I("\r\n2) change to T1, and move to %f, That is: \r\n", after_run_float_y1);
        LOG_I("T1 target coordinate in float: %f, in step: %d(%f = LROUND(%f * 80)\r\n",
              after_run_float_y1, after_run_int_y1, after_run_float_y1 * 80, after_run_float_y1);
        LOG_I("T1 current coordinate in float: %f, in step: %d(%f = LROUND(%f * 80)\r\n",
              origin_float_y1, origin_int_y1, origin_float_y1 * 80, origin_float_y1);
        LOG_I("T1 actual move in steps %d = (%d)(target steps) - (%d)(current steps)\r\n", T1_actual_move_steps, after_run_int_y1, origin_int_y1);

        LOG_I("\r\n3) Now change to T0, coordinate of T0(%f) = T1(%f) - offset(%f)\r\n", after_run_float_y0, after_run_float_y1, diff);
        LOG_I("And T0 move to origin: %f, that is current(%f) --> target(%f)\n", origin_float_y0, after_run_float_y0, origin_float_y0);
        LOG_I("T0 target coordinate in step: %d(LROUND(%f * 80)\r\n", origin_int_y0, origin_float_y0);
        LOG_I("T0 current coordinate in step: %d(LROUND(%f * 80)\r\n", after_run_int_y0, after_run_float_y0);
        LOG_I("T0 actual move in steps %d = %d(target steps) - %d(current steps)\r\n", T0_actual_move_steps, origin_int_y0, after_run_int_y0);

        LOG_I("Before tool change, T0 coordinate map<%f(planner), %d(stepper)>\r\n", origin_float_y0, origin_int_y0);
        LOG_I("After changing to T1, moving in T1, and than changing to T0 and return to T0 origin\r\n");
        LOG_I("T0 now coordinate map<%f(planner), %d(stepper)>\r\n", origin_float_y0, T0_after_move_step_pos);
        LOG_I("stepper position(%d) = T0_origin_step_pos(%d) + T1 move step(%d) + return T0 origin move steps(%d)\r\n",
              T0_after_move_step_pos, origin_int_y0, T1_actual_move_steps, T0_actual_move_steps);

        if (abs(origin_int_y0) != T0_after_move_step_pos) {
          LOG_I("\r\nERROR:\r\n");
        }
        else {
          LOG_I("\r\nOK:\r\n");
        }

        if (lc >= log_cnt) return;
      }
    }
  }
}
#endif

void setting_save_loop() {
  static uint32_t last_mills;
  if (PENDING(millis(), last_mills + 1000)) {
    return;
  }
  last_mills = millis();

  if (  ml_setting_need_save &&
        (!system_service.is_working()) &&
        (stepper.axis_did_move == 0)) {\
    LOG_I("J1 DELAY SAVE...\r\n");
    settings.save();
    ml_setting_need_save = false;
  }
}

void E_position_log(void) {
  static uint32_t last_ms = 0;
  if (PENDING(millis(), last_ms + 1000))
    return;

  last_ms = millis();
}

void j1_main_task(void *args) {

  uint32_t syslog_timeout = millis();
  log_reset_source();
  power_loss.show_power_loss_info();
  print_control.init();

  #if 0
  // LOG_I("LROUND(0.1) = %d, LROUND(0.5) = %d, LROUND(-0.1) = %d, LROUND(-0.5) = %d\n", LROUND(0.1), LROUND(0.5), LROUND(-0.1), LROUND(-0.5));
  // float_round_test(0.00125f, 1000);
  // float_round_test(-0.1375f, 20);
  #endif

  fd_srv.init();
  calibtration.updateBuildPlateThickness(fd_srv.getBuildPlateThickness());

  while(1) {
    print_control.loop();
    printer_event_loop();
    exception_event_loop();
    local_event_loop();
    E_position_log();

    axis_speed_update();
    sg_set();
    statistics_log();
    setting_save_loop();

    #if 0
    // probe_io_log();
    // TMC2209_log();
    // filament_sensor_log(0);
    // filament_sensor_log(1);
    #endif

    #if ENABLED(DEBUG_ISR_CPU_USAGE)
    step_isr_usage_log();
    #endif

    if (ELAPSED(millis(), syslog_timeout)) {
      syslog_timeout = millis() + 20000;
      LOG_V("%s: c0: %d/t0: %d, c1: %d/t1: %d, cb: %d/tb: %d, ", J1_BUILD_VERSION,
        (int)thermalManager.degHotend(0), thermalManager.degTargetHotend(0),
        (int)thermalManager.degHotend(1), thermalManager.degTargetHotend(1),
        (int)thermalManager.degBed(), thermalManager.degTargetBed());
      LOG_V("sta: %u, excep sta: 0x%x, excep beh: 0x%x\n", system_service.get_status(),
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

    // if (got_stepper_debug_info) {
    //   LOG_I("\r\n ======================================================= \r\n");
    //   LOG_I("stepper: %f %f %f\r\n", stepper_cur_position[X_AXIS], stepper_cur_position[Y_AXIS], stepper_cur_position[Z_AXIS]);
    //   LOG_I("motion_cur: %f %f %f\r\n", motion_cur_position[X_AXIS], motion_cur_position[Y_AXIS], motion_cur_position[Z_AXIS]);
    //   LOG_I("motion_tag: %f %f %f\r\n", motion_get_position[X_AXIS], motion_get_position[Y_AXIS], motion_get_position[Z_AXIS]);

    //   diff_x += (motion_cur_position[X_AXIS] - stepper_cur_position[X_AXIS]);
    //   diff_y += (motion_cur_position[Y_AXIS] - stepper_cur_position[Y_AXIS]);
    //   LOG_I("diff x: %f, diff y: %f\r\n", diff_x, diff_y);

    //   got_stepper_debug_info = false;
    // }
  }
}


void J1_setup() {
  update_server.init();
  switch_detect.init();
  fdm_head.init();
  debug.init();
  subscribe_init();
  event_init();
  system_service.init();

  TaskHandle_t thandle_j1_main = NULL;
  BaseType_t ret = xTaskCreate(j1_main_task, "j1_main_task", 1024, NULL, 5, &thandle_j1_main);
  if (ret != pdPASS) {
    SERIAL_ECHO("Failed to create j1_main_task!\n");
  }
  else {
    SERIAL_ECHO("Created j1_main_task task!\n");
  }
}
