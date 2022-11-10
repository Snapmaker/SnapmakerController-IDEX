#ifndef FILAMENT_SENSOR
#define FILAMENT_SENSOR

#include "stdint.h"

#define FILAMENT_SENSOR_COUNT 2
#define FILAMENT_LOOP(i) for (uint8_t i = 0; i < FILAMENT_SENSOR_COUNT; i++)
#define FILAMENT_CHECK_DISTANCE 2  // mm
#define FILAMENT_THRESHOLD 8  // ADC diff value
#define FILAMENT_ROUND_LEN 47 // mm
#define FILAMENT_CHECK_TIMES 5
#define SENSOR_DEAD_SPACE_ADC 1433  // > 1.51V
#define SENSOR_DEAD_SPACE_ADC_HW2 4050
#define SENSOR_DEAD_SPACE_ADC_MIN_HW2 30
#define SENSOR_DEAD_SPACE_DISTANCE 4  // mm
#define FILAMENT_THRESHOLD_HW2 15  // ADC diff value

typedef struct {
  bool enabled[FILAMENT_SENSOR_COUNT];
  float distance;  // Move this distance to detect abnormal sensing deviation
  uint8_t check_times;  // Determine the number of exceptions
  uint16_t threshold;
}filament_check_param_t;


class FilamentSensor
{
  public:
    void init();
    void e0_step(uint8_t step);
    void e1_step(uint8_t step);
    void next_sample(uint8_t e);
    void enable(uint8_t e) {
      filament_param.enabled[e] = true;
      triggered[e] = false;
      check_step_count[e] = 0;
      next_sample(e);
    }
    void disable(uint8_t e) {
      triggered[e] = false;
      filament_param.enabled[e] = false;
    }
    void enable_all() {
      FILAMENT_LOOP(i) {
        enable(i);
      }
    }
    void disable_all() {
      FILAMENT_LOOP(i) {
        disable(i);
      }
    }
    bool is_trigger(uint8_t e) {
      return triggered[e] && is_enable(e);
    }
    bool is_trigger() {
      return is_trigger(0) || is_trigger(1);
    }
    bool is_enable(uint8_t e) {
      return filament_param.enabled[e];
    }

    void check();
    void test_adc(uint8_t e, float step_mm, uint32_t count);
    void reset();
    void used_default_param();
    uint16_t get_adc_val(uint8_t e);
  public:
    filament_check_param_t filament_param;

  private:
    uint8_t err_mask = 0x1;
    int32_t check_step_count[FILAMENT_SENSOR_COUNT];
    int32_t check_adc_threshold[FILAMENT_SENSOR_COUNT];
    uint8_t err_times[FILAMENT_SENSOR_COUNT] = {0, 0};
    uint32_t e_step_count[FILAMENT_SENSOR_COUNT] = {0, 0};
    uint32_t last_e_step_count[FILAMENT_SENSOR_COUNT] = {0, 0};
    bool triggered[FILAMENT_SENSOR_COUNT] = {false, false};
    uint16_t start_adc[FILAMENT_SENSOR_COUNT] = {0, 0};
    int32_t e_step_statics_count[FILAMENT_SENSOR_COUNT] = {0, 0};

    uint32_t dead_space;
    uint32_t dead_space_min;
};

extern FilamentSensor filament_sensor;

#endif