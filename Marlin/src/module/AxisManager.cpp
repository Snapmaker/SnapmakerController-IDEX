#include "AxisManager.h"
#include "shaper/MoveQueue.h"
#include "../gcode/gcode.h"

#include "../../../snapmaker/J1/common_type.h"

#define DEFAULT_IS_FREQ   (50)
#define DEFAULT_IS_DAMP   (0.1)
#define DEFAULT_IS_TYPE   (1)

AxisManager axisManager;

const char* input_shaper_type_name[] = {"none", "ei", "ei2", "ei3", "mzv", "zv", "zvd", "zvdd", "zvddd"};

void AxisManager::input_shaper_reset() {

  AxisInputShaper::axis_input_shaper_x.type = (InputShaperType)DEFAULT_IS_TYPE;
  AxisInputShaper::axis_input_shaper_x.frequency = DEFAULT_IS_FREQ;
  AxisInputShaper::axis_input_shaper_x.zeta = DEFAULT_IS_DAMP;

  AxisInputShaper::axis_input_shaper_y.type = (InputShaperType)DEFAULT_IS_TYPE;
  AxisInputShaper::axis_input_shaper_y.frequency = DEFAULT_IS_FREQ;
  AxisInputShaper::axis_input_shaper_y.zeta = DEFAULT_IS_DAMP;

}

ErrCode AxisManager::input_shaper_set(int axis, int type, float freq, float dampe)  {

  if (axis != X_AXIS && axis != Y_AXIS) return E_PARAM;

  AxisInputShaper* axis_input_shaper = axisManager.axis[axis].axis_input_shaper;
  if (freq != axis_input_shaper->frequency || dampe != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
    axis_input_shaper->setConfig(type, freq, dampe);
    planner.synchronize();
    axisManager.initAxisShaper();
    axisManager.abort();
  }
  LOG_I("setting: axis: %d type: %s, frequency: %lf, zeta: %lf\n", axis, input_shaper_type_name[type], freq, dampe);

  return E_SUCCESS;
}

ErrCode AxisManager::input_shaper_get(int axis, int &type, float &freq, float &dampe) {

  if (axis != X_AXIS && axis != Y_AXIS) return E_PARAM;

  AxisInputShaper* axis_input_shaper = axisManager.axis[axis].axis_input_shaper;
  type = (int)axis_input_shaper->type;
  freq = axis_input_shaper->frequency;
  dampe = axis_input_shaper->zeta;
  // LOG_I("getting: axis: %d type: %s, frequency: %lf, zeta: %lf\n", axis, input_shaper_type_name[type], freq, dampe);

  return E_SUCCESS;
}

void GcodeSuite::M593() {
    LOG_I("M593\n");
    // if (axisManager.req_update_shaped) {
    //     LOG_I("Send too many\n");
    //     return;
    // }
    bool update = false;
    // if (parser.seen('P') || parser.seen('F') || parser.seen('D')) {
    //     update = true;
    // }
    bool x = parser.seen('X');
    bool y = parser.seen('Y');
    if (!x && !y) {
        x = true;
        y = true;
    }

    if (x) {
        AxisInputShaper* axis_input_shaper = axisManager.axis[0].axis_input_shaper;
        float frequency = parser.floatval('F', axis_input_shaper->frequency);
        float zeta = parser.floatval('D', axis_input_shaper->zeta);
        int type = parser.floatval('P', (int)axis_input_shaper->type);
        if (frequency != axis_input_shaper->frequency || zeta != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
            update = true;
        }
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
        if (frequency != axis_input_shaper->frequency || zeta != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
            update = true;
        }
        LOG_I("Y type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[type], frequency, zeta);
        if (!update) {
            axis_input_shaper->logParams();
        } else {
            axis_input_shaper->setConfig(type, frequency, zeta);
        }
    }
    LOG_I("update: %d\n", update);
    if (update) {
        planner.synchronize();
        axisManager.initAxisShaper();
        axisManager.abort();
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
    #if ENABLED(LIN_ADVANCE)
    } else if (axis == 3) {
        res = generateEAxisFuncParams(block_index, move_start, move_end);
    #endif
    } else {
      res = generateAxisFuncParams(move_start, move_end);
    }
    if (res) {
        generated_block_index = block_index;
    }

    return res;
}

bool Axis::getNextStep() {

    if (!func_manager.getNextPosTime(1, &dir, mm_to_step, half_step_mm)) {
      is_get_next_step_null = true;
      time_interval_valid = false;
      return false;
    }

    print_time = func_manager.print_time;
    if (time_interval_valid) {
      current_interval = print_time - last_print_time;
    }
    else {
      time_interval_valid = true;
      current_interval = .0;
    }
    last_print_time = print_time;
    is_consumed = false;

    return true;
}

float Axis::getCurrentSpeedMMs() {
  if (time_interval_valid && current_interval > 0.001)
    return 1000.0 / (current_interval * mm_to_step);
  else
    return 0;
}

#if ENABLED(LIN_ADVANCE)
FORCE_INLINE bool Axis::generateEAxisFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end) {
    uint8_t move_index;
    if (generated_move_index == -1) {
        move_index = move_start;
    } else {
        move_index = moveQueue.nextMoveIndex(generated_move_index);
    }

    while (move_index != moveQueue.nextMoveIndex(move_end)) {
        Move *move = &moveQueue.moves[move_index];

        if (IS_ZERO(move->t)) {
          move_index = moveQueue.nextMoveIndex(move_index);
          continue;
        }

        // #define K (0.04)
        float K = planner.block_buffer[block_index].use_advance_lead ? planner.extruder_advance_K[active_extruder] * 1000 : 0;
        float delta_v = IS_ZERO(move->accelerate) ? 0 : K * move->accelerate;
        float eda = delta_v * move->t * move->axis_r[axis];

        // LOG_I("k: %lf, d_v: %lf, eda: %lf\n", K, delta_v, eda);

        float a = 0.5f * move->accelerate * move->axis_r[axis];
        float b, c;

        if (delta_v < 0 && move->start_v + delta_v > 0 && move->end_v + delta_v < 0) {
            float zero_t = ABS((move->start_v + delta_v) / move->accelerate);
            float zero_pos = ((move->start_v + delta_v) * zero_t + 0.5f * move->accelerate * sq(zero_t)) * move->axis_r[axis];

            float y2 = move->start_pos[axis] + delta_e + zero_pos;
            float dy = zero_pos;
            float x2 = zero_t;
            float dx = zero_t;

            c = move->start_pos[axis] + delta_e;
            b = dy / dx - a * x2;

            int type;
            if (IS_ZERO(dy)) {
                type = 0;
            } else {
                type = dy > 0 ? 1 : -1;
            }

            time_double_t end_t = move->start_t + zero_t;
            func_manager.addFuncParams(a, b, c, type, end_t, y2);

            y2 = move->end_pos[axis] + delta_e + eda;
            dy = move->end_pos[axis] - move->start_pos[axis] + eda - zero_pos;
            x2 = move->t - zero_t;
            dx = move->t - zero_t;

            c = move->start_pos[axis] + delta_e + zero_pos;
            b = dy / dx - a * x2;

            if (IS_ZERO(dy)) {
                type = 0;
            } else {
                type = dy > 0 ? 1 : -1;
            }

            end_t = move->end_t;
            func_manager.addFuncParams(a, b, c, type, end_t, y2);
        } else {
            float y2 = move->end_pos[axis] + delta_e + eda;
            float dy = move->end_pos[axis] - move->start_pos[axis] + eda;
            float x2 = move->t;
            float dx = move->t;

            c = move->start_pos[axis] + delta_e;
            b = dy / dx - a * x2;

            int type;
            if (IS_ZERO(dy)) {
                type = 0;
            } else {
                type = dy > 0 ? 1 : -1;
            }
            time_double_t end_t = move->end_t;
            func_manager.addFuncParams(a, b, c, type, end_t, y2);
        }

        delta_e += eda;
        move_index = moveQueue.nextMoveIndex(move_index);
    }

    generated_move_index = move_end;
    return true;
}
#endif


FORCE_INLINE bool Axis::generateAxisFuncParams(uint8_t move_start, uint8_t move_end) {
  uint8_t move_index;
  if (generated_move_index == -1) {
      move_index = move_start;
  } else {
      move_index = moveQueue.nextMoveIndex(generated_move_index);
  }

  while (move_index != moveQueue.nextMoveIndex(move_end)) {
    Move *move = &moveQueue.moves[move_index];

    if (IS_ZERO(move->t)) {
      move_index = moveQueue.nextMoveIndex(move_index);
      continue;
    }

    generateLineFuncParams(move);

    move_index = moveQueue.nextMoveIndex(move_index);
  }
  generated_move_index = move_end;
  return true;
}


// FORCE_INLINE void Axis::generateLineFuncParams(Move* move) {
//     float y2 = move->end_pos[axis];
//     float dy = move->end_pos[axis] - move->start_pos[axis];
//     float x2 = move->t;
//     float dx = move->t;

//     float a = 0.5f * move->accelerate * move->axis_r[axis];
//     float c = move->start_pos[axis];
//     float b = dy / dx - a * x2;

//     int type;
//     if (IS_ZERO(dy)) {
//         type = 0;
//     } else {
//         type = dy > 0 ? 1 : -1;
//     }
//     time_double_t end_t = move->end_t;
//     func_manager.addFuncParams(a, b, c, type, end_t, y2);
// }

float AxisManager::getRemainingConsumeTime() {
    return min_last_time - print_time;
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

    // LOG_I("start %d, end %d\n", move_start, move_end);

    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (!axis[i].generateFuncParams(block_index, move_start, move_end)) {
            res = false;
        }
    }

    uint8_t new_move_tail = moveQueue.calculateMoveStart(move_start, axisManager.shaped_delta_window);

    moveQueue.updateMoveTail(new_move_tail);

    axisManager.updateMinLastTime();

    return res;
}

// /*
//  Copy next step's information to axis_stepper
//  and mark is_consumed = true;
// */
// bool AxisManager::getNextAxisStepper(AxisStepper *axis_stepper) {
//     if (getAxisStepperSize() == 0) { 
//         return false;
//     }

//     AxisStepper* current_stepper = &axis_steppers[axis_steppper_tail];
//     axis_stepper->axis = current_stepper->axis;
//     axis_stepper->dir = current_stepper->dir;
//     axis_stepper->delta_time = current_stepper->delta_time;
//     axis_stepper->print_time = current_stepper->print_time;

//     if (print_axis != T0_T1_AXIS_INDEX) {
//         current_steps[print_axis] += print_dir;
//     }

//     axis_steppper_tail = nextAxisStepper(axis_steppper_tail);
//     return true;
// }

/*
 Calcuation all axes's next step's time if need
 And then return the closest time axis if we have
*/
bool AxisManager::calcNextAxisStepper() {
    if (getAxisStepperFreeSize() == 0) {
        return false;
    }

    // If a axis has been consumed, calculate the next step time
    for (int i = 0; i < AXIS_SIZE; ++i) {
        if (axis[i].is_consumed) {
            axis[i].getNextStep();
        }
    }

    bool is_consumed = true;

    // Fine the closest time of all the axes
    time_double_t min_print_time = 2000000000;
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

        print_dir = axis[print_axis].dir;

        AxisStepper* axis_stepper = &axis_steppers[axis_steppper_head];

        axis_stepper->axis = print_axis;
        axis_stepper->dir = print_dir;
        axis_stepper->delta_time = min_print_time - print_time;
        axis_stepper->print_time = min_print_time;

        print_time = min_print_time;

        axis_steppper_head = nextAxisStepper(axis_steppper_head);

        return true;
    } else {
        return false;
    }
}

