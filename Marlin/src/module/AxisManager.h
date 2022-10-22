#pragma once

#include "planner.h"
#include "shaper/AxisInputShaper.h"
#include "shaper/FuncManager.h"
#include "shaper/MoveQueue.h"
#include "../../../../snapmaker/debug/debug.h"

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

    float delta_e = 0;

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
    }

    bool getNextStep();

  private:
    FORCE_INLINE bool generateAxisFuncParams(uint8_t move_start, uint8_t move_end);

    #if ENABLED(LIN_ADVANCE)
    FORCE_INLINE bool generateEAxisFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end);
    #endif

    FORCE_INLINE void generateLineFuncParams(Move* move);
};

class AxisManager {
  public:
    int counts[20] = {0};

    Axis axis[AXIS_SIZE];

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
    bool is_consumed = true;
    time_double_t print_time = 0;
    int8_t print_axis = -1;
    int8_t print_dir = 0;
    int current_steps[AXIS_SIZE];

  public:
    AxisManager() {};

    void init() {
        req_abort = false;

        for (int i = 0; i < AXIS_SIZE; ++i) {
            axis[i].init(i, planner.settings.axis_steps_per_mm[i]);
        }

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

        need_add_move_start = true;

        min_last_time = 0;

        is_consumed = true;
        print_time = 0;
        print_axis = -1;
        print_dir = 0;
    }

    void abort() {
        req_abort = true;
        moveQueue.reset();
        reset();
        addEmptyMove();
        req_abort = false;
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

    bool getCurrentAxisStepper(AxisStepper* axis_stepper);

    bool getNextAxisStepper();
};

extern AxisManager axisManager;


