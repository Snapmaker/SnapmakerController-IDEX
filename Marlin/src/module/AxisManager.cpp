#include "AxisManager.h"
#include "shaper/MoveQueue.h"

AxisManager axisManager;
AxisConsumerManager axisConsumerManager;

FORCE_INLINE bool Axis::generateFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end) {
    if (block_index == generated_block_index) {
        return true;
    }
    // is_get_next_step_null = false;

    bool res;
    if (is_shaped) {
        res = axis_input_shaper->generateShapedFuncParams(&func_manager, move_start, move_end);
    } else {
        res = generateAxisFuncParams(&func_manager, move_start, move_end);
    }
    if (res) {
        generated_block_index = block_index;
    }

    return res;
}

bool AxisConsumer::getNextStep(FuncParams* funcParams, int size, int func_params_head) {
    // if (is_get_next_step_null) {
        // return false;
    // }
    // if (func_manager.max_size < func_manager.getSize()) {
        // func_manager.max_size = func_manager.getSize();
    // }
    time_double_t* next_print_time = func_consumer.getNextPosTime(funcParams, size, func_params_head, 1, &dir, mm_to_step, half_step_mm);
    if (next_print_time == nullptr) {
        is_get_next_step_null = true;
        return false;
    }
    print_time = *next_print_time;
    is_consumed = false;
    return true;
}

FORCE_INLINE bool Axis::generateAxisFuncParams(FuncManager *func_manager, uint8_t move_start, uint8_t move_end) {
    uint8_t move_index;
    if (generated_move_index == -1) {
        move_index = move_start;
    } else {
        move_index = moveQueue.nextMoveIndex(generated_move_index);
    }

    Move *move;
    while (move_index != moveQueue.nextMoveIndex(move_end)) {
        move = &moveQueue.moves[move_index];

        if (IS_ZERO(move->t)) {
            move_index = moveQueue.nextMoveIndex(move_index);
            continue;
        }

        float y2 = move->end_pos[axis];
        float dy = move->end_pos[axis] - move->start_pos[axis];
        float x2 = move->t;
        float dx = move->t;

        float a = 0.5f * move->accelerate * move->axis_r[axis];
        float c = move->start_pos[axis];
        float b = dy / dx - a * x2;

        int type;
        if (IS_ZERO(dy)) {
            type = 0;
        } else {
            type = dy > 0 ? 1 : -1;
        }
        time_double_t end_t = move->end_t;
        func_manager->addFuncParams(a, b, c, type, end_t, y2);

        // LOG_I("%d, %d, %d\n", axis, func_manager.max_size, func_manager.func_params_head);

        move_index = moveQueue.nextMoveIndex(move_index);
    }
    generated_move_index = move_end;
    return true;
}

float AxisManager::getRemainingConsumeTime() {
    return min_last_time - axisConsumerManager.print_time;
}

bool AxisManager::generateAllAxisFuncParams(uint8_t block_index, block_t* block) {
    bool res = true;

    uint8_t move_start = block->shaper_data.move_start;
    uint8_t move_end = block->shaper_data.move_end;

    if (isShaped()) {
        if (move_start != moveQueue.move_tail && moveQueue.moves[moveQueue.prevMoveIndex(move_start)].flag == 1) {
            move_start = moveQueue.prevMoveIndex(move_start);
        }

        if (move_end != moveQueue.prevMoveIndex(moveQueue.move_head) && moveQueue.moves[moveQueue.nextMoveIndex(move_end)].flag == 1) {
            move_end = moveQueue.nextMoveIndex(move_end);
        }
        if (move_end == moveQueue.move_head) {
            move_end = moveQueue.prevMoveIndex(moveQueue.move_head);
        }
    }

    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (!axis[i].generateFuncParams(block_index, move_start, move_end)) {
            res = false;
        }
    }

    uint8_t new_move_tail = moveQueue.calculateMoveStart(move_start, axisManager.shaped_delta);

    moveQueue.updateMoveTail(new_move_tail);

    axisManager.updateMinLastTime();

    return res;
}

/*
 Copy next step's information to axis_stepper
 and mark is_consumed = true;
*/
bool AxisConsumerManager::getCurrentAxisStepper(AxisStepper *axis_stepper) {
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
bool AxisConsumerManager::getNextAxisStepper() {
    if (!is_consumed) {
        return false;
    }

    // If a axis has been consumed, calculate the next step time
    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (axis_consumers[i].is_consumed) {
            FuncManager& func_manager = axisManager.axis[i].func_manager;
            FuncParams* func_params = func_manager.funcParams;
            // int func_params_use = func_manager.func_params_use;
            int func_params_head = func_manager.func_params_head;
            int size = func_manager.size;
            axis_consumers[i].getNextStep(func_params, size, func_params_head);
        }
    }

    // Fine the closest time of all the axes
    time_double_t min_print_time = 1000000000;
    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (!axis_consumers[i].is_consumed) {
            if (axis_consumers[i].print_time < min_print_time) {
                min_print_time = axis_consumers[i].print_time;
                print_axis = i;
                is_consumed = false;
            }
        }
    }

    // is_consumed == false, means that we has a steps need to output
    if (!is_consumed) {
        axis_consumers[print_axis].is_consumed = true;
        print_time = min_print_time;
        print_dir = axis_consumers[print_axis].dir;
        return true;
    } else {
        return false;
    }
}

