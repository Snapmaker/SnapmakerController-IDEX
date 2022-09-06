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
    // if (is_get_next_step_null) {
        // return false;
    // }
    // if (func_manager.max_size < func_manager.getSize()) {
        // func_manager.max_size = func_manager.getSize();
    // }
    time_double_t* next_print_time = func_manager.getNextPosTime(1, &dir, mm_to_step, half_step_mm);
    if (next_print_time == nullptr) {
        is_get_next_step_null = true;
        return false;
    }
    print_time = *next_print_time;
    is_consumed = false;
    return true;
}

bool Axis::generateFuncParams(FuncManager &func_manager, uint8_t move_start, uint8_t move_end) {
    uint8_t move_index;
    if (generated_move_index == -1) {
        move_index = move_start;
    } else {
        move_index = moveQueue.nextMoveIndex(generated_move_index);
    }

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
    generated_move_index = move_end;
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

        if (move_end != moveQueue.prevMoveIndex(moveQueue.move_head) && moveQueue.moves[moveQueue.nextMoveIndex(move_end)].flag == 1) {
            move_end = moveQueue.nextMoveIndex(move_end);
        }
    }

    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (!axis[i].generateFuncParams(block_index, block, move_start, move_end)) {
            res = false;
        }
    }

    moveQueue.updateMoveTail(moveQueue.calculateMoveStart(move_start, axisManager.shaped_delta));

    axisManager.updateMinLastTime();

    return res;
}

/*
 Copy next step's information to axis_stepper
 and mark is_consumed = true;
*/
bool AxisManager::getCurrentAxisStepper(AxisStepper *axis_stepper) {
    if (is_consumed) {
        return false;
    }
    axis_stepper->axis = print_axis;
    axis_stepper->dir = print_dir;
    axis_stepper->print_time = print_time;

    current_steps[print_axis] += print_dir;

    is_consumed = true;
    return true;
}

/*
 Calcuation all axes's next step's time if need
 And then return the closest time axis if we have
*/
bool AxisManager::getNextAxisStepper() {
    if (getRemainingConsumeTime() == 0 || !is_consumed) {
        return false;
    }

    // If a axis has been consumed, calculate the next step time
    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (axis[i].is_consumed) {
            axis[i].getNextStep();
        }
    }

    // Fine the closest time of all the axes
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

    // is_consumed == false, means that we has a steps need to output
    if (!is_consumed) {
        axis[print_axis].is_consumed = true;
        print_time = min_print_time;
        print_dir = axis[print_axis].dir;
        return true;
    } else {
        return false;
    }
}

