#include "MoveQueue.h"
#include "../../../../snapmaker/debug/debug.h"

MoveQueue moveQueue;

static xyze_float_t ZERO_AXIS_R = {0};

void MoveQueue::calculateMoves(block_t* block) {
    float millimeters = block->millimeters;

    float entry_speed = block->initial_speed / 1000.0f;
    float leave_speed = block->final_speed / 1000.0f;
    float nominal_speed = block->nominal_speed / 1000.0f;

    if (nominal_speed < EPSILON) {
        // LOG_I("error speed: %lf\n", nominal_speed);
        block->shaper_data.is_zero_speed = true;
        return;
    }

    float i_nominal_speed = 1000.0f / block->nominal_speed;
    float acceleration = LROUND(block->acceleration) / 1000000.0f;
    float i_acceleration = 1000000.0f / LROUND(block->acceleration);

    float accelDistance = Planner::estimate_acceleration_distance(entry_speed, nominal_speed, acceleration);
    // if (accelDistance > millimeters + EPSILON) {
        // LOG_I("error accelDistance: %lf, %lf\n", accelDistance, millimeters);
    // }
    if (accelDistance < EPSILON) {
        accelDistance = 0;
    }
    float accelClocks = (nominal_speed - entry_speed) * i_acceleration;

    float deceleration = acceleration;
    float decelDistance = Planner::estimate_acceleration_distance(nominal_speed, leave_speed, -deceleration);
    // if (decelDistance > millimeters + EPSILON) {
        // LOG_I("error decelDistance: %lf, %lf\n", decelDistance, millimeters);
    // }
    if (decelDistance < EPSILON) {
        decelDistance = 0;
    }
    float decelClocks = (nominal_speed - leave_speed) * i_acceleration;

    float plateau = millimeters - accelDistance - decelDistance;

    if (plateau < 0) {
        float newAccelDistance = Planner::intersection_distance(entry_speed, leave_speed, acceleration, millimeters);
        if (newAccelDistance > millimeters + EPSILON) {
            // LOG_I("error newAccelDistance: %lf, %lf\n", newAccelDistance, millimeters);
        }
        if (newAccelDistance > millimeters) {
            newAccelDistance = millimeters;
        }
        if (newAccelDistance < EPSILON) {
            newAccelDistance = 0;
        }
        if ((millimeters - newAccelDistance) < EPSILON) {
            newAccelDistance = millimeters;
        }
        accelDistance = newAccelDistance;
        nominal_speed = SQRT(2 * acceleration * newAccelDistance + sq(entry_speed));
        if (nominal_speed < leave_speed) {
            nominal_speed = leave_speed;
        }
        accelClocks = (nominal_speed - entry_speed) * i_acceleration;
        decelDistance = millimeters - accelDistance;
        decelClocks = (nominal_speed - leave_speed) * i_acceleration;
        plateau = 0;
    }

    block->shaper_data.move_start = move_head;

    float plateauClocks = plateau * i_nominal_speed;

    xyze_float_t axis_r;
    axis_r.x = block->axis_r.x;
    axis_r.y = block->axis_r.y;
    axis_r.z = block->axis_r.z;
    axis_r.e = block->axis_r.e;

    if (plateau == 0) {
        if (accelDistance > 0) {
            addMove(entry_speed, nominal_speed, acceleration, accelDistance, axis_r, accelClocks);
        }
        if (decelDistance > 0) {
            addMove(nominal_speed, leave_speed, -deceleration, decelDistance, axis_r, decelClocks);
        }
    } else {
        if (accelDistance > 0) {
            addMove(entry_speed, nominal_speed, acceleration, accelDistance, axis_r, accelClocks);
        }

        // LOG_I("p: %lf, s: %lf, t: %lf\n", plateau, nominal_speed, plateau / nominal_speed);
        addMove(nominal_speed, nominal_speed, 0, plateau, axis_r, plateauClocks);

        if (decelDistance > 0) {
            addMove(nominal_speed, leave_speed, -deceleration, decelDistance, axis_r, decelClocks);
        }
    }

    block->shaper_data.block_time = accelClocks + plateauClocks + decelClocks;

    block->shaper_data.move_end = prevMoveIndex(move_head);

    block->curise_speed = nominal_speed * 1000;

    Move& end_move = moves[block->shaper_data.move_end];
    for (int i = 0; i < AXIS_SIZE; ++i) {
        float p1 = end_move.end_pos[i];
        end_move.end_pos[i] = LROUND(end_move.end_pos[i]);
        if (ABS(p1 - end_move.end_pos[i]) > 1) {
            LOG_I("error LROUND: %lf, %lf\n", p1, end_move.end_pos[i]);
        }
        // if (i == 0 || i == 1) {
            // LOG_I("%d %lf %lf\n", i, tmp, end_move.end_pos[i]);
        // }
    }

    block->shaper_data.last_print_time = moves[block->shaper_data.move_end].end_t;
}

void MoveQueue::setMove(uint8_t move_index, float start_v, float end_v, float accelerate, float distance, xyze_float_t& axis_r, float t, uint8_t flag) {
    Move &move = moves[move_index];

    move.start_v = start_v;
    move.end_v = end_v;

    move.accelerate = accelerate;
    move.distance = distance;

    move.t = t;
    move.axis_r[0] = axis_r.x;
    move.axis_r[1] = axis_r.y;
    move.axis_r[2] = axis_r.z;
    move.axis_r[3] = axis_r.e;

    Move& last_move = moves[prevMoveIndex(move_index)];
    move.start_t = is_first ? 0 : last_move.end_t;
    move.end_t = move.start_t + move.t;

    // LOG_I("move_index: %d %lf %d %lf\n", move_index, t, flag, move.start_t.toFloat());

    float last_end_v = is_first? 0 : last_move.end_v;
    if (!IS_ZERO(last_end_v - move.start_v)) {
        // LOG_I("error v: %lf, %lf\n", last_end_v,  move.start_v);
    }

    for (int i = 0; i < AXIS_SIZE; ++i) {
        move.start_pos[i] = is_first ? 0 : last_move.end_pos[i];
        move.end_pos[i] = move.start_pos[i] + move.distance * move.axis_r[i];

        if (IS_ZERO(move.end_pos[i]) && !IS_ZERO(move.start_pos[i])) {
            // LOG_I("debug1: %lf, %lf\n", move.start_pos[i], move.end_pos[i]);
        }

        if (i <= 1 && (move.end_pos[i] < -48000 || move.end_pos[i] > 48000)) {
            // LOG_I("debug: %d, %lf, %lf\n", i, move.distance, move.end_pos[i]);
        }
    }

    is_first = false;

    // LOG_I("v1: %lf, v2: %lf, s_p: %lf, e_p: %lf, t: %lf\n", start_v, end_v, move.start_pos[3], move.end_pos[3], t);

    // LOG_I("%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n", move_head, move.t, move.start_t.toFloat(), move.end_t.toFloat(), start_v, distance, move.start_pos[0], move.end_pos[0], move.start_pos[1], move.end_pos[1], move.flag);

    // LOG_I("move, %lf %lf %lf %lf %lf %lf\n", move.start_t.toDouble(), move.end_t.toDouble(), move.accelerate, move.axis_r[0], move.axis_r[1], move.distance);

//    if (flag != 1) {
//        Move& last_move = moves[prevMoveIndex(move_index)];
//        for (int i = 0; i < AXIS_SIZE; ++i) {
//            move.start_pos[i] = last_move.end_pos[i];
//            move.end_pos[i] = move.start_pos[i] + move.distance * move.axis_r[i];
//        }
//    } else {
//        for (int i = 0; i < AXIS_SIZE; ++i) {
//            move.start_pos[i] = 0;
//            move.end_pos[i] = move.start_pos[i] + move.distance * move.axis_r[i];
//        }
//    }

    move.flag = flag;

    // printf("move: end_t: %lf, pos: %lf, start_v: %lf, t: %lf, flag: %d\n", move.end_t.toDouble(), move.end_pos[0], move.start_v, move.t, move.flag);
}

uint8_t MoveQueue::addMove(float start_v, float end_v, float accelerate, float distance, xyze_float_t& axis_r, float t, uint8_t flag) {
    setMove(move_head, start_v, end_v, accelerate, distance, axis_r, t, flag);

    uint8_t move_index = move_head;

    move_head = nextMoveIndex(move_head);

//    setMoveEnd();

    return move_index;
}

uint8_t MoveQueue::addEmptyMove(float time) {
    return addMove(0, 0, 0, 0, ZERO_AXIS_R, time, MOVE_FLAG_START);
}

uint8_t MoveQueue::addMoveStart() {
    return addMove(0, 0, 0, 0, ZERO_AXIS_R, EMPTY_TIME, MOVE_FLAG_START);
}

uint8_t MoveQueue::addMoveEnd() {
    return addMove(0, 0, 0, 0, ZERO_AXIS_R, EMPTY_TIME, MOVE_FLAG_END);
}

void MoveQueue::setMoveEnd() {
    setMove(move_head, 0, 0, 0, 0, ZERO_AXIS_R, EMPTY_TIME, MOVE_FLAG_END);
}


uint8_t MoveQueue::calculateMoveStart(uint8_t index, float delta_window) {
    if (index == move_tail) {
        return index;
    }
    uint8_t move_shaped_start = prevMoveIndex(index);
    float t = moves[move_shaped_start].t;

    while (t < delta_window && move_shaped_start != move_tail) {
        move_shaped_start = prevMoveIndex(move_shaped_start);
        t += moves[move_shaped_start].t;
    }
    return move_shaped_start;
};

uint8_t MoveQueue::calculateMoveEnd(uint8_t index, float delta_window) {
    if (index == move_head) {
        return index;
    }
    uint8_t move_shaped_end = nextMoveIndex(index);
    float t = moves[move_shaped_end].t;

    while (t < delta_window && move_shaped_end != move_head) {
        move_shaped_end = nextMoveIndex(move_shaped_end);
        t += moves[move_shaped_end].t;
    }
    return move_shaped_end;
};

void MoveQueue::updateMoveTail(uint8_t index) {
    move_tail = index;
};

void MoveQueue::initMoveTimeAndPos(uint8_t move_shaped_start, uint8_t move_start, uint8_t move_shaped_end) {
    float start_t = 0;
    float start_pos[AXIS_SIZE] = {0};

    uint8_t index = move_start;
    Move *move;
    while (index != nextMoveIndex(move_shaped_end)) {
        move = &moves[index];

        move->start_t = start_t;
        start_t += move->t;
        move->end_t = start_t;

//        for (int i = 0; i < AXIS_SIZE; ++i) {
//            move->start_pos[i] = start_pos[i];
//            start_pos[i] += move->distance * move->axis_r[i];
//            move->end_pos[i] = start_pos[i];
//        }

        index = nextMoveIndex(index);
    }

    start_t = 0;

//    for (int i = 0; i < AXIS_SIZE; ++i) {
//        start_pos[i] = 0;
//    }

    index = prevMoveIndex(move_start);

    while (index != prevMoveIndex(move_shaped_start)) {
        move = &moves[index];

        move->end_t = start_t;
        start_t -= move->t;
        move->start_t = start_t;

//        for (int i = 0; i < AXIS_SIZE; ++i) {
//            move->end_pos[i] = start_pos[i];
//            start_pos[i] -= move->distance * move->axis_r[i];
//            move->start_pos[i] = start_pos[i];
//        }

        index = prevMoveIndex(index);
    }
}

float MoveQueue::getAxisPositionAcrossMoves(int move_index, int axis, time_double_t time, int move_shaped_start, int move_shaped_end) {
    while (time < moves[move_index].start_t && move_index != move_shaped_start) {
        move_index = prevMoveIndex(move_index);
    }
    while (time > moves[move_index].end_t && move_index != move_shaped_end) {
        move_index = nextMoveIndex(move_index);
    }

    return getAxisPosition(move_index, axis, time);
}

float MoveQueue::getAxisPosition(int move_index, int axis, time_double_t time) {
    Move *move = &moves[move_index];
    float axis_r = move->axis_r[axis];
    float start_pos = move->start_pos[axis];

    float delta_time = time - move->start_t;

    float move_dist = (move->start_v + 0.5f * move->accelerate * delta_time) * delta_time;

    return start_pos + axis_r * move_dist;
}