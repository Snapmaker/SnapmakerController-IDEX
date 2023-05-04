#pragma once

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

#include "planner.h"
#include "shaper/AxisInputShaper.h"
#include "shaper/FuncManager.h"
#include "shaper/MoveQueue.h"
#include "../../../../snapmaker/debug/debug.h"
#include "../../../../snapmaker/J1/common_type.h"

#define T0_T1_AXIS_INDEX  (4)

#define AXIS_STEPPER_SIZE 4
#define AXIS_STEPPER_MOD(n) ((n)&(AXIS_STEPPER_SIZE-1))

enum InputShaperDebugInfoType {
  SHAPER_DBG_EMPTY_MOVES_COUNT = 0,
  SHAPER_DBG_NO_STEPS,
  SHAPER_DBG_NOT_ENOUGH_MOVES_RESC,
  SHAPER_DBG_NOT_ENOUGH_FUNC_LIST_RESC,
  SHAPER_DBG_CALC_STEP_TIMEOUT_COUNT,
  SHAPER_DBG_CALC_STEP_TIME,
  SHAPER_DBG_ABORT_END_BLOCK,

  SHAPER_DBG_MAX
};

class AxisStepper {
  public:
    int8_t axis = -1;
    int8_t last_axis = -1;
    int8_t dir = 0;
    time_double_t print_time = 0;
    float delta_time = 0;
};

class Axis {
  public:
    float mm_to_step;
    float half_step_mm;

    // AxisInputShaper
    AxisInputShaper* axis_input_shaper = nullptr;
    bool is_shaped = false;

    // FuncManager
    FuncManager func_manager;

    // Consume
    bool is_consumed = true;
    time_double_t print_time = 0;
    int8_t dir = 0;
    bool is_get_next_step_null = false;
    float current_interval;
    float cur_speed;
    time_double_t last_print_time = 0;
    bool time_interval_valid = false;

    double delta_e = 0;

  private:
    int8_t axis;

    // Generate
    int generated_block_index = -1;
    int generated_move_index = -1;


  public:
    Axis() {};

    void init(int8_t axis, float mm_to_step) {
        this->axis = axis;
        this->func_manager.init(axis);
        this->mm_to_step = mm_to_step;
        this->half_step_mm = 0.5 / mm_to_step;
        this->func_manager.reset();
    }

    void initShaper() {
        if (axis > 1) {
            return;
        }
        if (axis_input_shaper == nullptr && axis == 0) {
            axis_input_shaper = &AxisInputShaper::axis_input_shaper_x;
            axis_input_shaper->setAxis(axis);
        }
        if (axis_input_shaper == nullptr && axis == 1) {
            axis_input_shaper = &AxisInputShaper::axis_input_shaper_y;
            axis_input_shaper->setAxis(axis);
        }
        axis_input_shaper->init();
        is_shaped = axis_input_shaper->isShaped();
    }

    FORCE_INLINE bool generateFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end);

    void reset() {
        generated_block_index = -1;
        generated_move_index = -1;

        is_consumed = true;
        print_time = 0;
        dir = 0;
        is_get_next_step_null = false;

        delta_e = 0;

        if (axis_input_shaper != nullptr) {
            axis_input_shaper->reset();
        }

        func_manager.reset();

        current_interval = .0;
        time_interval_valid = false;
    }

    bool getNextStep();
    float getCurrentSpeedMMs();

    FORCE_INLINE void generateLineFuncParams(Move* move) {
        float y2 = move->end_pos[axis];
        float dy = move->end_pos[axis] - move->start_pos[axis];
        float x2 = move->t;
        float dx = move->t;

        float a = 0.5f * move->accelerate * move->axis_r[axis];
        float c = move->start_pos[axis];
        float b = dy / dx - a * x2;

        // LOG_I("a %f b %f c %f\r\n", a, b, c);

        int type;
        if (IS_ZERO(dy)) {
            type = 0;
        } else {
            type = dy > 0 ? 1 : -1;
        }
        time_double_t end_t = move->end_t;
        func_manager.addFuncParams(a, b, c, type, end_t, y2);
    }

  private:
    FORCE_INLINE bool generateAxisFuncParams(uint8_t move_start, uint8_t move_end);

    #if ENABLED(LIN_ADVANCE)
    FORCE_INLINE bool generateEAxisFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end);
    #endif

};

class AxisManager {
  public:
    int counts[20] = {0};
    bool T0_T1_simultaneously_move_req = false;
    bool T0_T1_simultaneously_move = false;
    float T0_T1_target_pos;
    int32_t T0_T1_execute_steps;
    int32_t T0_T1_calc_steps;
    int32_t inactive_x_step_pos;
    int32_t X0_home_step_pos;
    int32_t X1_home_step_pos;
    uint8_t T0_T1_axis = 0;
    time_double_t T0_T1_last_print_time = 0;

    Axis axis[AXIS_SIZE];
    Axis axis_t0_t1;

    volatile bool req_abort;

    // MoveQueue
    bool need_add_move_start = true;

    // AxisInputShaper
    bool is_shaped = false;
    float shaped_left_delta = 0;
    float shaped_right_delta = 0;
    float shaped_delta_window = 0;

    // FuncManager Generate
    time_double_t min_last_time = 0;

    // FuncManager Consume
    // bool is_consumed = true;
    time_double_t print_time = 0;
    int8_t print_axis = -1;
    int8_t print_dir = 0;
    int current_steps[AXIS_SIZE];

    AxisStepper axis_steppers[AXIS_STEPPER_SIZE];
    uint8_t axis_steppper_tail;
    uint8_t axis_steppper_head;

    FORCE_INLINE uint8_t getAxisStepperSize() {
        return AXIS_STEPPER_MOD(axis_steppper_head - axis_steppper_tail);
    }

    FORCE_INLINE uint8_t getAxisStepperFreeSize() { return AXIS_STEPPER_SIZE - 1 - getAxisStepperSize(); }

    FORCE_INLINE uint8_t nextAxisStepper(uint8_t index) {
        return AXIS_STEPPER_MOD(index + 1);
    }

    FORCE_INLINE uint8_t prevAxisStepper(uint8_t index) {
        return AXIS_STEPPER_MOD(index - 1);
    }

  public:
    void input_shaper_reset();
    ErrCode input_shaper_set(int axis, int type, float freq, float dampe);
    ErrCode input_shaper_get(int axis, int &type, float &freq, float &dampe);
    void show_debug_info();
    void reset_debug_info();

    AxisManager() {};

    void init() {
        req_abort = false;

        for (int i = 0; i < AXIS_SIZE; ++i) {
            axis[i].init(i, planner.settings.axis_steps_per_mm[i]);
        }

        axis_t0_t1.init(T0_T1_AXIS_INDEX, planner.settings.axis_steps_per_mm[X_AXIS]);

        initAxisShaper();

        addEmptyMove();
    };

    void initAxisShaper() {
        is_shaped = false;
        for (int i = 0; i < AXIS_SIZE; ++i) {
            axis[i].initShaper();

            if (axis[i].is_shaped) {
                is_shaped = true;
            }
        }

        shaped_left_delta = 0;
        shaped_right_delta = 0;
        shaped_delta_window = 0;

        if (is_shaped) {
            for (int i = 0; i < AXIS_SIZE; ++i) {
                if (axis[i].axis_input_shaper != nullptr && axis[i].axis_input_shaper->left_delta > shaped_left_delta) {
                    shaped_left_delta = axis[i].axis_input_shaper->left_delta;
                }
                if (axis[i].axis_input_shaper != nullptr && axis[i].axis_input_shaper->right_delta > shaped_right_delta) {
                    shaped_right_delta = axis[i].axis_input_shaper->right_delta;
                }
                if (axis[i].axis_input_shaper != nullptr && axis[i].axis_input_shaper->delta_window > shaped_delta_window) {
                    shaped_delta_window = axis[i].axis_input_shaper->delta_window;
                }
            }
        }
    }

    void reset() {
        for (size_t i = 0; i < AXIS_SIZE; i++) {
            axis[i].reset();
            current_steps[i] = 0;
        }

        axis_t0_t1.reset();

        need_add_move_start = true;

        min_last_time = 0;

        print_time = 0;
        print_axis = -1;
        print_dir = 0;

        axis_steppper_tail = 0;
        axis_steppper_head = 0;
    }

    void abort() {
        req_abort = true;
        moveQueue.reset();
        reset();
        addEmptyMove();
    }

    bool isShaped() {
        return is_shaped;
    }

    bool generateAllAxisFuncParams(uint8_t block_index, block_t* block);

    float getRemainingConsumeTime();

    bool tryAddMoveStart() {
        if (!isShaped() || !need_add_move_start) {
            return false;
        }
        moveQueue.addMoveStart();
        need_add_move_start = false;
        return true;
    }

    void updateMinLastTime() {
        time_double_t new_min_last_time = axis[0].func_manager.last_time;
        for (int i = 1; i < AXIS_SIZE; ++i) {
            if (axis[i].func_manager.last_time < new_min_last_time) {
                new_min_last_time = axis[i].func_manager.last_time;
            }
        }
        min_last_time = new_min_last_time;
    }

    bool tryAddMoveEnd() {
        if (!isShaped() || need_add_move_start) {
            return false;
        }
        moveQueue.addMoveEnd();
        need_add_move_start = true;
        return true;
    }

    int addEmptyMove() {
        if (!isShaped()) {
            return -1;
        }
        // LOG_I("shaped_delta_window: %lf\n", shaped_delta_window);
        return moveQueue.addEmptyMove(shaped_delta_window + 0.001f);
    }

    FORCE_INLINE bool getNextZeroAxisStepper(AxisStepper* axis_stepper) {
        if (getAxisStepperSize() == 0) {
            return false;
        }

        AxisStepper* current_stepper = &axis_steppers[axis_steppper_tail];

        if (current_stepper->delta_time > 0.005) {
            return false;
        }

        axis_stepper->axis = current_stepper->axis;
        axis_stepper->dir = current_stepper->dir;
        axis_stepper->delta_time = current_stepper->delta_time;
        axis_stepper->print_time = current_stepper->print_time;

        if (axis_stepper->axis != T0_T1_AXIS_INDEX) {
            current_steps[axis_stepper->axis] += axis_stepper->dir;
        }

        axis_steppper_tail = nextAxisStepper(axis_steppper_tail);
        return true;
    };

    FORCE_INLINE bool getNextAxisStepper(AxisStepper* axis_stepper) {
        if (getAxisStepperSize() == 0 && !calcNextAxisStepper()) {
            return false;
        }

        AxisStepper* current_stepper = &axis_steppers[axis_steppper_tail];
        axis_stepper->axis = current_stepper->axis;
        axis_stepper->dir = current_stepper->dir;
        axis_stepper->delta_time = current_stepper->delta_time;
        axis_stepper->print_time = current_stepper->print_time;

        if (axis_stepper->axis != T0_T1_AXIS_INDEX) {
            current_steps[axis_stepper->axis] += axis_stepper->dir;
        }

        axis_steppper_tail = nextAxisStepper(axis_steppper_tail);
        return true;
    };

    bool calcNextAxisStepper();
};


extern AxisManager axisManager;


