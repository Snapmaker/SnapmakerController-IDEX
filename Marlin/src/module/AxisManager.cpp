#include "AxisManager.h"
#include "shaper/MoveQueue.h"
#include "../gcode/gcode.h"

AxisManager axisManager;

void GcodeSuite::M593() {
    LOG_I("M593\n");
    if (axisManager.req_update_shaped) {
        LOG_I("Send too many\n");
        return;
    }
    bool update = false;
    if (parser.seen('P') || parser.seen('F') || parser.seen('D')) {
        update = true;
    }
    bool x = parser.seen('X');
    bool y = parser.seen('Y');
    if (!x && !y) {
        x = true;
        y = true;
    }
    LOG_I("update: %d\n", update);
    const char* input_shaper_type_name[] = {"none", "ei", "ei2", "ei3", "mzv", "zv", "zvd", "zvdd", "zvddd"};
    if (x) {
        AxisInputShaper* axis_input_shaper = axisManager.axis[0].axis_input_shaper;
        float frequency = parser.floatval('F', axis_input_shaper->frequency);
        float zeta = parser.floatval('D', axis_input_shaper->zeta);
        int type = parser.floatval('P', (int)axis_input_shaper->type);
        LOG_I("X type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[type], frequency, zeta);
        if (!update) {
            axis_input_shaper->logParams();
        } else {
            axis_input_shaper->setConfig(type, frequency, zeta);
        }
    }
    if (y) {
        AxisInputShaper* axis_input_shaper = axisManager.axis[1].axis_input_shaper;
        float frequency = parser.floatval('F', axis_input_shaper->frequency);
        float zeta = parser.floatval('D', axis_input_shaper->zeta);
        int type = parser.floatval('P', (int)axis_input_shaper->type);
        LOG_I("Y type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[type], frequency, zeta);
        if (!update) {
            axis_input_shaper->logParams();
        } else {
            axis_input_shaper->setConfig(type, frequency, zeta);
        }
    }
    if (update) {
        if(planner.movesplanned() == 0) {
            axisManager.initAxisShaper();
            axisManager.addEmptyMove();
        } else {
            axisManager.req_update_shaped = true;
        }
    }
}

FORCE_INLINE bool Axis::generateFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end) {
    if (block_index == generated_block_index) {
        return true;
    }
    // is_get_next_step_null = false;

    bool res;
    if (is_shaped) {
        res = axis_input_shaper->generateShapedFuncParams(&func_manager, move_start, move_end);
    } else {
        res = generateAxisFuncParams(move_start, move_end);
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
    if (!func_manager.getNextPosTime(1, &dir, mm_to_step, half_step_mm)) {
        is_get_next_step_null = true;
        return false;
    }
    print_time = func_manager.print_time;
    is_consumed = false;
    return true;
}

FORCE_INLINE bool Axis::generateAxisFuncParams(uint8_t move_start, uint8_t move_end) {
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
        func_manager.addFuncParams(a, b, c, type, end_t, y2);

        // LOG_I("%d, %d, %d\n", axis, func_manager.max_size, func_manager.func_params_head);

        move_index = moveQueue.nextMoveIndex(move_index);
    }
    generated_move_index = move_end;
    return true;
}

float AxisManager::getRemainingConsumeTime() {
    return min_last_time - print_time;
}

bool AxisManager::generateAllAxisFuncParams(uint8_t block_index, block_t* block) {
    bool res = true;

    uint8_t move_start = block->shaper_data.move_start;
    uint8_t move_end = block->shaper_data.move_end;

    if (req_update_shaped && req_update_index == -1) {
        if (moveQueue.back().flag != 1) {
            req_update_index = addEmptyMove();
        }
    }

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

    // LOG_I("start %d, end %d\n", move_start, move_end);

    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (!axis[i].generateFuncParams(block_index, move_start, move_end)) {
            res = false;
        }
    }

    if (req_update_shaped && req_update_index == move_end) {
        initAxisShaper();
        addEmptyMove();
        req_update_shaped = false;
        req_update_index = -1;
    }

    uint8_t new_move_tail = moveQueue.calculateMoveStart(move_start, axisManager.shaped_delta_window);

    moveQueue.updateMoveTail(new_move_tail);

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
    if (!is_consumed) {
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

