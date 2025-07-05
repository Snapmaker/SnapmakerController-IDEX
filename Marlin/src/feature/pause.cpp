/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * feature/pause.cpp - Pause feature support functions
 * This may be combined with related G-codes if features are consolidated.
 */

 #include "../inc/MarlinConfigPre.h"

 #if ENABLED(ADVANCED_PAUSE_FEATURE) && !IS_HMI_PRINTING
 
 #include "../MarlinCore.h"
 #include "../gcode/gcode.h"
 #include "../module/motion.h"
 #include "../module/planner.h"
 #include "../module/stepper.h"
 #include "../module/printcounter.h"
 #include "../module/temperature.h"
 
 #if ENABLED(FWRETRACT)
   #include "fwretract.h"
 #endif
 
 #if HAS_FILAMENT_SENSOR
   #include "runout.h"
 #endif
 
 #if ENABLED(HOST_ACTION_COMMANDS)
   #include "host_actions.h"
 #endif
 
 #if ENABLED(EXTENSIBLE_UI)
   #include "../lcd/extui/ui_api.h"
 #endif
 
 #include "../lcd/marlinui.h"
 
 #if HAS_BUZZER
   #include "../libs/buzzer.h"
 #endif
 
 #if ENABLED(POWER_LOSS_RECOVERY)
   #include "powerloss.h"
 #endif
 
 #include "../libs/nozzle.h"
 #include "pause.h"
 
 #define DEBUG_OUT ENABLED(DEBUG_PAUSE_RESUME)
 #include "../core/debug_out.h"
 
 // Private variables
 static xyze_pos_t resume_position;
 
 #if M600_PURGE_MORE_RESUMABLE
   PauseMenuResponse pause_menu_response;
   PauseMode pause_mode = PAUSE_MODE_PAUSE_PRINT;
 #endif
 
 fil_change_settings_t fc_settings[EXTRUDERS];
 
 #if ENABLED(SDSUPPORT)
   #include "../sd/cardreader.h"
 #endif
 
 #if ENABLED(EMERGENCY_PARSER)
   #define _PMSG(L) L##_M108
 #else
   #define _PMSG(L) L##_LCD
 #endif
 
 #if HAS_BUZZER
   static void impatient_beep(const int8_t max_beep_count, const bool restart = false) {
     if (TERN0(HAS_LCD_MENU, pause_mode == PAUSE_MODE_PAUSE_PRINT)) return;
 
     static millis_t next_buzz = 0;
     static int8_t runout_beep = 0;
 
     if (restart) next_buzz = runout_beep = 0;
 
     const bool always = max_beep_count < 0;
     const millis_t ms = millis();
     if (ELAPSED(ms, next_buzz)) {
       if (always || runout_beep < max_beep_count + 5) {
         next_buzz = ms + ((always || runout_beep < max_beep_count) ? 1000 : 500);
         BUZZ(50, 880 - (runout_beep & 1) * 220);
         runout_beep++;
       }
     }
   }
   inline void first_impatient_beep(const int8_t max_beep_count) { impatient_beep(max_beep_count, true); }
 #else
   inline void impatient_beep(const int8_t, const bool = false) {}
   inline void first_impatient_beep(const int8_t) {}
 #endif
 
 bool ensure_safe_temperature(const bool wait, const PauseMode mode) {
   #if ENABLED(PREVENT_COLD_EXTRUSION)
     if (!DEBUGGING(DRYRUN) && thermalManager.targetTooColdToExtrude(active_extruder))
       thermalManager.setTargetHotend(thermalManager.extrude_min_temp, active_extruder);
   #endif
 
   ui.pause_show_message(PAUSE_MESSAGE_HEATING, mode);  // Unused by Snapmaker, but kept
   return thermalManager.wait_for_hotend(active_extruder, wait);
 }
 
 bool load_filament(const_float_t slow_load_length, const_float_t fast_load_length, const_float_t purge_length, const int8_t max_beep_count,
                    const bool show_lcd, const bool pause_for_user, const PauseMode mode DXC_ARGS) {
   if (!ensure_safe_temperature(false, mode)) {
     if (show_lcd) ui.pause_show_message(PAUSE_MESSAGE_STATUS, mode);
     return false;
   }
 
   if (pause_for_user) {
     SERIAL_ECHO_MSG(_PMSG(STR_FILAMENT_CHANGE_INSERT));
     first_impatient_beep(max_beep_count);
     KEEPALIVE_STATE(PAUSED_FOR_USER);
     wait_for_user = true;
     #if ENABLED(HOST_PROMPT_SUPPORT)
       const char tool = '0' + TERN0(MULTI_FILAMENT_SENSOR, active_extruder);
       host_prompt_do(PROMPT_USER_CONTINUE, PSTR("Load Filament T"), tool, CONTINUE_STR);
     #endif
     while (wait_for_user) {
       impatient_beep(max_beep_count);
       idle_no_sleep();
     }
   }
 
   if (slow_load_length) unscaled_e_move(slow_load_length, FILAMENT_CHANGE_SLOW_LOAD_FEEDRATE);
   if (fast_load_length) unscaled_e_move(fast_load_length, FILAMENT_CHANGE_FAST_LOAD_FEEDRATE);
   if (purge_length > 0) unscaled_e_move(purge_length, ADVANCED_PAUSE_PURGE_FEEDRATE);
 
   TERN_(HOST_PROMPT_SUPPORT, host_action_prompt_end());
   return true;
 }
 
 inline void disable_active_extruder() {
   #if HAS_E_STEPPER_ENABLE
     disable_e_stepper(active_extruder);
     safe_delay(100);
   #endif
 }
 
 bool unload_filament(const_float_t unload_length, const bool show_lcd,
                      const PauseMode mode
                      #if BOTH(FILAMENT_UNLOAD_ALL_EXTRUDERS, MIXING_EXTRUDER)
                        , const_float_t mix_multiplier
                      #endif
 ) {
   #if !BOTH(FILAMENT_UNLOAD_ALL_EXTRUDERS, MIXING_EXTRUDER)
     constexpr float mix_multiplier = 1.0f;
   #endif
 
   if (!ensure_safe_temperature(false, mode)) return false;
 
   unscaled_e_move(-(FILAMENT_UNLOAD_PURGE_RETRACT) * mix_multiplier, (PAUSE_PARK_RETRACT_FEEDRATE) * mix_multiplier);
   safe_delay(FILAMENT_UNLOAD_PURGE_DELAY);
   unscaled_e_move((FILAMENT_UNLOAD_PURGE_RETRACT + FILAMENT_UNLOAD_PURGE_LENGTH) * mix_multiplier,
                   (FILAMENT_UNLOAD_PURGE_FEEDRATE) * mix_multiplier);
 
   #if FILAMENT_CHANGE_UNLOAD_ACCEL > 0
     const float saved_acceleration = planner.settings.retract_acceleration;
     planner.settings.retract_acceleration = FILAMENT_CHANGE_UNLOAD_ACCEL;
   #endif
 
   unscaled_e_move(unload_length * mix_multiplier, (FILAMENT_CHANGE_UNLOAD_FEEDRATE) * mix_multiplier);
 
   #if FILAMENT_CHANGE_FAST_LOAD_ACCEL > 0
     planner.settings.retract_acceleration = saved_acceleration;
   #endif
 
   disable_active_extruder();
   return true;
 }
 
 // Public variables and functions
 uint8_t did_pause_print = 0;
 bool is_m600_pause = false;
 
 bool pause_print(const_float_t retract, const xyz_pos_t &park_point, const bool show_lcd, const_float_t unload_length DXC_ARGS) {
   if (did_pause_print) return false;
 
   #if ENABLED(HOST_ACTION_COMMANDS)
     #ifdef ACTION_ON_PAUSED
       host_action_paused();
     #elif defined(ACTION_ON_PAUSE)
       host_action_pause();
     #endif
   #endif
 
   #if ENABLED(HOST_PROMPT_SUPPORT)
     host_prompt_open(PROMPT_INFO, PSTR("Pause"), DISMISS_STR);
   #endif
 
   ++did_pause_print;
 
   #if ENABLED(SDSUPPORT)
     const bool was_sd_printing = IS_SD_PRINTING();
     if (was_sd_printing) {
       card.pauseSDPrint();
       ++did_pause_print;
     }
   #endif
 
   print_job_timer.pause();
 
   resume_position = current_position;
   const bool do_park = !axes_should_home();
 
   #if ENABLED(POWER_LOSS_RECOVERY)
     const float park_raise = do_park ? nozzle.park_mode_0_height(park_point.z) - current_position.z : POWER_LOSS_ZRAISE;
     if (was_sd_printing && recovery.enabled) recovery.save(true, park_raise, do_park);
   #endif
 
   planner.synchronize();
 
   #if ENABLED(ADVANCED_PAUSE_FANS_PAUSE) && HAS_FAN
     thermalManager.set_fans_paused(true);
   #endif
 
   if (retract && thermalManager.hotEnoughToExtrude(active_extruder))
     unscaled_e_move(retract, PAUSE_PARK_RETRACT_FEEDRATE);
 
   if (do_park) nozzle.park(0, park_point);
 
   #if ENABLED(DUAL_X_CARRIAGE)
     const int8_t saved_ext = active_extruder;
     const bool saved_ext_dup_mode = extruder_duplication_enabled;
     set_duplication_enabled(false, DXC_ext);
   #endif
 
   if (unload_length)
     unload_filament(unload_length, show_lcd, PAUSE_MODE_CHANGE_FILAMENT);
 
   #if ENABLED(DUAL_X_CARRIAGE)
     set_duplication_enabled(saved_ext_dup_mode, saved_ext);
   #endif
 
   disable_active_extruder();
   return true;
 }
 
 void show_continue_prompt(const bool is_reload) {
   ui.pause_show_message(is_reload ? PAUSE_MESSAGE_INSERT : PAUSE_MESSAGE_WAITING);
   SERIAL_ECHO_START();
   SERIAL_ECHOPGM_P(is_reload ? PSTR(_PMSG(STR_FILAMENT_CHANGE_INSERT) "\n") : PSTR(_PMSG(STR_FILAMENT_CHANGE_WAIT) "\n"));
 }
 
 void wait_for_confirmation(const bool is_reload, const int8_t max_beep_count DXC_ARGS) {
   bool nozzle_timed_out = false;
   const millis_t nozzle_timeout = SEC_TO_MS(PAUSE_PARK_NOZZLE_TIMEOUT);
   HOTEND_LOOP() thermalManager.heater_idle[e].start(nozzle_timeout);
 
   KEEPALIVE_STATE(PAUSED_FOR_USER);
   wait_for_user = true;
 
   show_continue_prompt(is_reload);
   first_impatient_beep(max_beep_count);
 
   #if ENABLED(HOST_PROMPT_SUPPORT)
     host_prompt_do(PROMPT_USER_CONTINUE, PSTR("Nozzle Parked - Ready to Resume Print?"), CONTINUE_STR);
   #endif
 
   while (wait_for_user) {
     impatient_beep(max_beep_count);
     HOTEND_LOOP() nozzle_timed_out |= thermalManager.heater_idle[e].timed_out;
     if (nozzle_timed_out) {
       #if ENABLED(HOST_PROMPT_SUPPORT)
         host_action_prompt_end();  // Close "Nozzle Parked" prompt
         host_prompt_do(PROMPT_USER_CONTINUE, PSTR("Heater Timed Out - Reheat?"), PSTR("Reheat"));
       #endif
       SERIAL_ECHOLNPGM("Heater timed out - waiting for reheat confirmation");
       wait_for_user = true;
       while (wait_for_user) idle_no_sleep();
       SERIAL_ECHOLNPGM("Reheating nozzle");
       HOTEND_LOOP() thermalManager.reset_hotend_idle_timer(e);
       thermalManager.setTargetHotend(thermalManager.temp_hotend[active_extruder].target, active_extruder);
       ensure_safe_temperature(false);
       #if ENABLED(HOST_PROMPT_SUPPORT)
         host_prompt_do(PROMPT_USER_CONTINUE, PSTR("Nozzle Parked - Ready to Resume Print?"), CONTINUE_STR);
       #endif
       HOTEND_LOOP() thermalManager.heater_idle[e].start(nozzle_timeout);
       wait_for_user = true;
       nozzle_timed_out = false;
       first_impatient_beep(max_beep_count);
     }
     idle_no_sleep();
   }
 }
 
 void resume_print(const_float_t slow_load_length, const_float_t fast_load_length, const_float_t purge_length, const int8_t max_beep_count, const celsius_t targetTemp DXC_ARGS) {
   if (!did_pause_print) return;
 
   bool nozzle_timed_out = false;
   HOTEND_LOOP() {
     nozzle_timed_out |= thermalManager.heater_idle[e].timed_out;
     thermalManager.reset_hotend_idle_timer(e);
   }
 
   if (targetTemp > thermalManager.degTargetHotend(active_extruder))
     thermalManager.setTargetHotend(targetTemp, active_extruder);
 
   load_filament(slow_load_length, fast_load_length, purge_length, max_beep_count, true, nozzle_timed_out, PAUSE_MODE_SAME DXC_PASS);
 
   if (targetTemp > 0) {
     thermalManager.setTargetHotend(targetTemp, active_extruder);
     thermalManager.wait_for_hotend(active_extruder, false);
   }
 
   ensure_safe_temperature();
 
   unscaled_e_move(-(PAUSE_PARK_RETRACT_LENGTH), feedRate_t(PAUSE_PARK_RETRACT_FEEDRATE));
 
   if (!axes_should_home()) {
     destination.set(resume_position.x, resume_position.y, current_position.z, current_position.e);
     prepare_internal_move_to_destination(NOZZLE_PARK_XY_FEEDRATE);
     destination.z = resume_position.z;
     prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);
   }
 
   unscaled_e_move(PAUSE_PARK_RETRACT_LENGTH, feedRate_t(PAUSE_PARK_RETRACT_FEEDRATE));
 
   #if ENABLED(FWRETRACT)
     if (fwretract.retracted[active_extruder])
       unscaled_e_move(-fwretract.settings.retract_length, fwretract.settings.retract_feedrate_mm_s);
   #endif
 
   if (resume_position.e < 0) unscaled_e_move(resume_position.e, feedRate_t(PAUSE_PARK_RETRACT_FEEDRATE));
   #if ADVANCED_PAUSE_RESUME_PRIME != 0
     unscaled_e_move(ADVANCED_PAUSE_RESUME_PRIME, feedRate_t(ADVANCED_PAUSE_PURGE_FEEDRATE));
   #endif
 
   planner.set_e_position_mm((destination.e = current_position.e = resume_position.e));
 
   #if defined(ACTION_ON_RESUME)
    host_action_resume();  
   #elif defined(ACTION_ON_RESUMED)
    host_action_resumed();
  #endif
 
   --did_pause_print;
 
   #if ENABLED(HOST_PROMPT_SUPPORT)
     host_prompt_open(PROMPT_INFO, PSTR("Resuming"), DISMISS_STR);
   #endif
 
   if (print_job_timer.isPaused()) print_job_timer.start();
 
   #if ENABLED(SDSUPPORT)
     if (did_pause_print) {
       --did_pause_print;
       card.startOrResumeFilePrinting();
       TERN_(POWER_LOSS_RECOVERY, if (recovery.enabled) recovery.save(true));
     }
   #endif
 
   #if ENABLED(ADVANCED_PAUSE_FANS_PAUSE) && HAS_FAN
     thermalManager.set_fans_paused(false);
   #endif
 
   TERN_(HAS_FILAMENT_SENSOR, runout.reset());
 }
 
 #endif // ADVANCED_PAUSE_FEATURE && !IS_HMI_PRINTING