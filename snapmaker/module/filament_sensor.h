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

 #ifndef FILAMENT_SENSOR_H
 #define FILAMENT_SENSOR_H
 
 #include "../../Marlin/src/inc/MarlinConfig.h"  // For Marlin types and NUM_RUNOUT_SENSORS
 #include "stdint.h"
 
 #define FILAMENT_SENSOR_COUNT 2
 #define FILAMENT_LOOP(i) for (uint8_t i = 0; i < FILAMENT_SENSOR_COUNT; i++)
 #define FILAMENT_CHECK_DISTANCE 2  // mm
 #define FILAMENT_CHECK_EXTRAS_DISTANCE  1  // mm
 #define FILAMENT_THRESHOLD 8  // ADC diff value
 #define FILAMENT_THRESHOLD_HW2 15  // ADC diff value
 #define FILAMENT_CHECK_TIMES 3
 #define SENSOR_DEAD_SPACE_ADC 1433  // > 1.51V
 #define SENSOR_DEAD_SPACE_ADC_HW2 4060
 #define SENSOR_DEAD_SPACE_ADC_MIN_HW2 60
 #define SENSOR_DEAD_SPACE_DISTANCE 4  // mm
 
 typedef struct {
   bool enabled[FILAMENT_SENSOR_COUNT];
   float distance;  // Move this distance to detect abnormal sensing deviation
   uint8_t check_times;  // Determine the number of exceptions
   uint16_t threshold;
 } filament_check_param_t;
 
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
     void reset();                   // Resets all sensors
     void reset(uint8_t e);          // Resets specific sensor
     void used_default_param();
     uint16_t get_adc_val(uint8_t e);
 
   public:
     filament_check_param_t filament_param;
 
   private:
     uint16_t err_mask = 0x1;  // Changed to uint16_t for 16-bit mask
     int32_t check_step_count[FILAMENT_SENSOR_COUNT];
     uint16_t err_times[FILAMENT_SENSOR_COUNT] = {0, 0};  // Changed to uint16_t
     int32_t e_step_count[FILAMENT_SENSOR_COUNT] = {0, 0};
     bool triggered[FILAMENT_SENSOR_COUNT] = {false, false};
     uint16_t start_adc[FILAMENT_SENSOR_COUNT] = {0, 0};
 };
 
 extern FilamentSensor filament_sensor;
 
 #endif // FILAMENT_SENSOR_H