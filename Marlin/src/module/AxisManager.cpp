#include "AxisManager.h"
#include "shaper/MoveQueue.h"

AxisManager axisManager;

bool Axis::generateFuncParams(uint8_t block_index, block_t &block, uint8_t move_start, uint8_t move_end) {
    if (block_index == generated_block_index) {
        return true;
    }
    is_get_next_step_null = false;

    bool res;
    if (is_shaped) {
        res = axis_input_shaper->generateFuncParams(func_manager, move_start, move_end);
    } else {
        res = generateFuncParams(func_manager, move_start, move_end);
    }
    if (res) {
        generated_block_index = block_index;
    }

    return res;
}

bool Axis::getNextStep() {
    if (is_get_next_step_null) {
        return false;
    }
    float next_step = func_manager.print_pos == 0 ? half_step : step;
    time_double_t* next_print_time = func_manager.getNextPosTime(step, &dir);
    if (next_print_time == nullptr) {
        is_get_next_step_null = true;
        return false;
    }
    print_time = *next_print_time;
    is_consumed = false;
    return true;
}

bool Axis::generateFuncParams(FuncManager &func_manager, uint8_t move_start, uint8_t move_end) {
    uint8_t move_index = move_start;
    Move *move;
    while (move_index != moveQueue.nextMoveIndex(move_end)) {
        move = &moveQueue.moves[move_index];
        float axis_r = move->axis_r[axis];

        float a = 0.5 * move->accelerate * axis_r;
        float b = move->start_v * axis_r;
        float c = move->start_pos[axis];

        func_manager.addDeltaTimeFuncParams(a, b, c, move->start_t, move->end_t, move->end_pos[axis]);

        move_index = moveQueue.nextMoveIndex(move_index);
    }
    return true;
}

bool AxisManager::generateAllAxisFuncParams(uint8_t block_index, block_t& block) {
    bool res = true;

    uint8_t move_start = block.shaper_data.move_start;
    uint8_t move_end = block.shaper_data.move_end;

    if (isShaped()) {
        if (move_start != moveQueue.move_tail && moveQueue.moves[moveQueue.prevMoveIndex(move_start)].flag == 1) {
            move_start = moveQueue.prevMoveIndex(move_start);
        }

        if (move_end != moveQueue.move_head && moveQueue.moves[moveQueue.nextMoveIndex(move_end)].flag == 1) {
            move_end = moveQueue.nextMoveIndex(move_end);
        }
    }

    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (!axis[i].generateFuncParams(block_index, block, move_start, move_end)) {
            res = false;
        }
    }

    axisManager.updateMinLastTime();

    return res;
}

bool AxisManager::getCurrentAxisStepper(AxisStepper *axis_stepper) {
    if (is_consumed) {
        return false;
    }
    axis_stepper->axis = print_axis;
    axis_stepper->dir = print_dir;
    axis_stepper->print_time = print_time;

    is_consumed = true;
    return true;
}

bool AxisManager::getNextAxisStepper() {
    if (getRemainingConsumeTime() == 0 || !is_consumed) {
        return false;
    }

    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (axis[i].is_consumed) {
            axis[i].getNextStep();
        }
    }

    time_double_t min_print_time = 1000000000;

    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (!axis[i].is_consumed) {
            if (axis[i].print_time < min_print_time) {
                min_print_time = axis[i].print_time;
                print_axis = i;
                is_consumed = false;
            }
        }
    }

    if (!is_consumed) {
        axis[print_axis].is_consumed = true;
        print_time = min_print_time;
        print_dir = axis[print_axis].dir;
        return true;
    } else {
        return false;
    }
}

