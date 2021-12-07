#ifndef FILAMENT_SENSOR
#define FILAMENT_SENSOR

#include "stdint.h"

#define FILAMENT_SENSOR_COUNT 2
#define FILAMENT_LOOP(i) for (uint8_t i = 0; i < FILAMENT_SENSOR_COUNT; i++)
#define FILAMENT_BASE_LEN 3  // mm
#define FILAMENT_SAMPLE_COUNT_MASK 0x3
#define FILAMENT_THRESHOLD 10  // ADC diff value

typedef struct {
  uint32_t scale;
  uint32_t tar_scale;
  uint32_t e_steps;
}strFILErrorRecord;

class FilamentSample {
  public:
    // Was temperature isr updated  
    inline void sample(const uint32_t s) { raw += s; sample_num++;}
    void ready() {
      if (sample_num) {
        value = raw / sample_num;
      }
      sample_num = raw = 0;
    }
    uint16_t get() {return value;}
  private:
    int32_t raw = 0;
    uint16_t value;
    uint8_t sample_num = 0;
};

class FilamentSensor
{
  public:
    void init();
    void e0_step(uint8_t step);
    void e1_step(uint8_t step);
    void next_sample(uint8_t e);
    void ready() {
      FILAMENT_LOOP(i) {
        filament[i].ready();
      }
    }
    void enable(uint8_t e) {
      enable_flag[e] = true;
      error[e] = 0;
      next_sample(e);
    }
    void disable(uint8_t e) {
      error[e] = 0;
      enable_flag[e] = false;
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
      return error[e];
    }
    bool is_enable(uint8_t e) {
      return enable_flag[e];
    }

    void debug();
    void check();
    void test_adc(uint8_t e, float step_mm, uint32_t count);
  public:
    FilamentSample filament[FILAMENT_SENSOR_COUNT];
  private:
    int32_t check_step_count[FILAMENT_SENSOR_COUNT];
    bool enable_flag[FILAMENT_SENSOR_COUNT] = {false, false};
    int32_t e_step_count[FILAMENT_SENSOR_COUNT] = {0, 0};
    uint8_t error[FILAMENT_SENSOR_COUNT];
    bool status[FILAMENT_SENSOR_COUNT];
    uint16_t start_adc[FILAMENT_SENSOR_COUNT] = {0, 0};
    int8_t filament_dir[FILAMENT_SENSOR_COUNT] = {1, -1};
};

extern FilamentSensor filament_sensor;

#endif