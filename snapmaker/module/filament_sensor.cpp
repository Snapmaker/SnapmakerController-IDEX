// /**
//  * Marlin 3D Printer Firmware
//  */
#include "HAL.h"
#include "../../Marlin/src/pins/pins.h"
#include "../../Marlin/src/core/serial.h"
#include "../../Marlin/src/module/planner.h"
#include "filament_sensor.h"
#include "system.h"
#include "motion_control.h"
#include "../debug/debug.h"

FilamentSensor filament_sensor;

void FilamentSensor::init() {
  pinMode(FILAMENT0_ADC_PIN, INPUT_ANALOG);
  pinMode(FILAMENT1_ADC_PIN, INPUT_ANALOG);

  dead_space = HW_1_2(SENSOR_DEAD_SPACE_ADC, SENSOR_DEAD_SPACE_ADC_HW2);
  dead_space_min = HW_1_2(0, SENSOR_DEAD_SPACE_ADC_MIN_HW2);
  filament_param.threshold = HW_1_2(FILAMENT_THRESHOLD, FILAMENT_THRESHOLD_HW2);

  reset();
}

uint16_t FilamentSensor::get_adc_val(uint8_t e) {
  uint32_t val = 0;
  uint8_t times = 3;
  for (uint8_t i = 0; i < times; i++) {
    if (e == 0) {
      val += analogRead(FILAMENT0_ADC_PIN);
    } else {
      val += analogRead(FILAMENT1_ADC_PIN);
    }
  }
  return val / times;
}

void FilamentSensor::reset() {

  // LOG_I("filament sensor reset\r\n");

  FILAMENT_LOOP(i) {
    start_adc[i] = 0;
    triggered[i] = false;
    err_times[i] = 0;
    check_step_count[i] = filament_param.distance * planner.settings.axis_steps_per_mm[E_AXIS_N(i)];
    float adc_per_step = (float)(dead_space - dead_space_min) / (FILAMENT_ROUND_LEN * planner.settings.axis_steps_per_mm[E_AXIS_N(i)]);
    check_adc_threshold[i] = check_step_count[i] * adc_per_step * 0.5;
    // LOG_I("E%d adc_per_step %f, check_step_count %d, check_adc_threshold %d\r\n",
    //       i, adc_per_step, check_step_count[i], check_adc_threshold[i]);
  }

  err_mask = ~(0xff << filament_param.check_times);
}

void FilamentSensor::used_default_param() {
  FILAMENT_LOOP(i) {
    filament_param.enabled[i] = true;
  }
  filament_param.check_times = FILAMENT_CHECK_TIMES;
  filament_param.distance = FILAMENT_CHECK_DISTANCE;
  reset();
}

void FilamentSensor::e0_step(uint8_t step) {
  if (step) {
    e_step_count[0]++;
    e_step_statics_count[0]++;
  } else {
    // e_step_count[0]--;
  }
}

void FilamentSensor::e1_step(uint8_t step) {
  if (step) {
    e_step_count[1]++;
  } else {
    // e_step_count[1]--;
  }
}

void FilamentSensor::next_sample(uint8_t e) {
  last_e_step_count[e] = e_step_count[e];
  // e_step_count[e] = 0;
  start_adc[e] = get_adc_val(e);
}

void FilamentSensor::check() {
  static int32_t dead_space_times[FILAMENT_SENSOR_COUNT] = {0, 0};
  FILAMENT_LOOP(i) {

    if (!is_enable(i)) {
      continue;
    }

    if (start_adc[i] == 0) {
      // The value is assigned once at startup
      next_sample(i);
      continue;
    }

    // {
    //   static uint32_t last_log_tick_ms = 0;
    //   if (ELAPSED(millis(), last_log_tick_ms + 200)) {
    //     uint16_t adc = get_adc_val(0);
    //     LOG_I("E0 adc: %d\r\n", adc);

    //     adc = get_adc_val(1);
    //     LOG_I("E1 adc: %d\r\n", adc);
    //     last_log_tick_ms = millis();
    //   }
    // }

    // {
    //   static uint8_t cs = 0;
    //   static uint32_t low_dead_space_steps = 0;
    //   static uint32_t high_dead_space_steps = 0;
    //   static uint32_t mid_space_steps = 0;
    //   static int32_t last_e_step_count = 0;

    //   if (e_step_statics_count[0] != last_e_step_count) {

    //     uint32_t delta = e_step_statics_count[0] - last_e_step_count;
    //     last_e_step_count = e_step_statics_count[0];
    //     uint16_t adc = get_adc_val(0);
    //     if (adc < 10) {
    //       if (cs != 0) {
    //         LOG_I("mid_space_steps %d\r\n", mid_space_steps);
    //         mid_space_steps = 0;
    //       }
    //       low_dead_space_steps += delta;
    //       cs = 0;
    //     }
    //     else if (adc > 4070) {
    //       if (cs != 1) {
    //         LOG_I("low_dead_space_steps %d\r\n", low_dead_space_steps);
    //         low_dead_space_steps = 0;
    //       }
    //       high_dead_space_steps += delta;
    //       cs = 1;
    //     }
    //     else {
    //       if (cs != 2) {
    //         LOG_I("high_dead_space_steps %d\r\n", high_dead_space_steps);
    //         high_dead_space_steps = 0;
    //       }
    //       mid_space_steps += delta;
    //       cs = 2;
    //     }
    //   }

    // }

    if (PENDING(e_step_count[i], last_e_step_count[i] + check_step_count[i]))
      continue;

    // if (e_step_count[i] < check_step_count[i])
    //   continue;

    uint16_t adc = get_adc_val(i);
    int32_t diff = abs(adc - start_adc[i]);
    // uint32_t dead_space = HW_1_2(SENSOR_DEAD_SPACE_ADC, SENSOR_DEAD_SPACE_ADC_HW2);
    // uint32_t dead_space_min = HW_1_2(0, SENSOR_DEAD_SPACE_ADC_MIN_HW2);
    // filament_param.threshold = HW_1_2(FILAMENT_THRESHOLD, FILAMENT_THRESHOLD_HW2);
    LOG_V("T%d adc:%d diff:%d TH:%d DS:%d\n", i, adc, diff, filament_param.threshold, dead_space);

    bool is_err = (diff < filament_param.threshold);
    // bool is_err = diff < check_adc_threshold[i];
    if ((adc > dead_space) || (adc < dead_space_min)) {
      dead_space_times[i]++;
      LOG_I("E%d filament dead space %d\r\n", i, dead_space_times[i]);
      if (dead_space_times[i] >= (3 * SENSOR_DEAD_SPACE_DISTANCE / FILAMENT_CHECK_DISTANCE)) {
        dead_space_times[i] = 0;
        err_times[i] = err_mask;
      }
    }
    else {
      if (is_err) {
        LOG_I("E%d diff %d\r\n", i, diff);
      }
      dead_space_times[i] = 0;
      err_times[i] = (err_times[i] << 1) | is_err;
    }

    // bool is_err = (diff < filament_param.threshold);
    // if (is_err &&
    //     (start_adc[i] > dead_space) || (start_adc[i] < dead_space_min) &&
    //     (adc > dead_space) || (adc < dead_space_min)) {
    //   dead_space_times[i]++;
    //   if (dead_space_times[i] >= (filament_param.check_times + (SENSOR_DEAD_SPACE_DISTANCE / FILAMENT_CHECK_DISTANCE))) {
    //     dead_space_times[i] = 0;
    //     err_times[i] = err_mask;
    //   }
    // } else {
    //   dead_space_times[i] = 0;
    //   err_times[i] = err_times[i] << 1 | is_err;
    // }

    if ((err_times[i] & err_mask) == err_mask) {
      triggered[i] = true;
      LOG_I("extruder %d blocked\r\n", i);
    }
    else {
      triggered[i] = false;
    }

    next_sample(i);

  }
}

void FilamentSensor::test_adc(uint8_t e, float step_mm, uint32_t count) {
  uint16_t max = 0x0, min=0xffff;
  uint32_t acc = 0, time=0;
  if (e >= FILAMENT_SENSOR_COUNT) {
    return;
  }
  uint16_t last_adc = get_adc_val(e);
  SERIAL_ECHOLNPAIR("tast filament sensor ", e);
  for (uint32_t i = 0; i < count; i++) {
    motion_control.extrude_e(step_mm, 15 * 60);
    planner.synchronize();
    time = millis();
    while (PENDING(millis(), (time + 8)));
    uint16_t adc = get_adc_val(e);
    int32_t diff = adc - last_adc;
    if (diff > 500 || diff < -500) {
      continue;
    }
    last_adc = adc;
    SERIAL_ECHOLNPAIR("diff:", diff);
    SERIAL_ECHOLNPAIR("rawadc:", adc);
    if (diff < min) {
      min = diff;
    }
    if (diff > max) {
      max = diff;
    }
    acc += diff;
  }
  SERIAL_ECHOLNPAIR("max:", max, ", min:", min, ", avr:", acc / count);
}
