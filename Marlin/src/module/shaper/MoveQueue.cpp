#include "MoveQueue.h"
#include "../../../../snapmaker/debug/debug.h"

MoveQueue moveQueue;

static xyze_float_t ZERO_AXIS_R = {0};

void MoveQueue::calculateMoves(block_t &block) {
//    if (is_start) {
//        block.shaper_data.is_start = true;
//        addMoveStart();
//        is_start = false;
//    }
    float speed_factor = block.nominal_speed / block.nominal_rate;
    float millimeters = block.millimeters;

    float entry_speed = block.initial_rate * speed_factor / 1000;
    float leave_speed = block.final_rate * speed_factor / 1000;
    float nominal_speed = block.nominal_speed / 1000;
    float acceleration = block.acceleration / 1000000;

    float accelDistance = Planner::estimate_acceleration_distance(entry_speed, nominal_speed, acceleration);

    float accelClocks = (nominal_speed - entry_speed) / acceleration;

    float deceleration = acceleration;
    float decelDistance = Planner::estimate_acceleration_distance(nominal_speed, leave_speed, -deceleration);

    float decelClocks = (nominal_speed - leave_speed) / deceleration;

    float plateau = millimeters - accelDistance - decelDistance;

    if (plateau < 0) {
        float newAccelDistance = Planner::intersection_distance(entry_speed, leave_speed, acceleration, millimeters);
        accelDistance = newAccelDistance;
        nominal_speed = SQRT(2 * acceleration * newAccelDistance + sq(entry_speed));
        if (nominal_speed < leave_speed) {
            nominal_speed = leave_speed;
        }
        accelClocks = (nominal_speed - entry_speed) / acceleration;
        decelDistance = millimeters - accelDistance;
        decelClocks = (nominal_speed - leave_speed) / deceleration;
        plateau = 0;
    }

    block.shaper_data.move_start = move_head;

    if (plateau == 0) {
        if (accelDistance > 0) {
            addMove(entry_speed, nominal_speed, acceleration, accelDistance, block.axis_r, accelClocks);
        }
        if (decelDistance > 0) {
            addMove(nominal_speed, leave_speed, -deceleration, decelDistance, block.axis_r, decelClocks);
        }
    } else {
        if (accelDistance > 0) {
            addMove(entry_speed, nominal_speed, acceleration, accelDistance, block.axis_r, accelClocks);
        }

        // LOG_I("p: %lf, s: %lf, t: %lf\n", plateau, nominal_speed, plateau / nominal_speed);
        addMove(nominal_speed, nominal_speed, 0, plateau, block.axis_r, plateau / nominal_speed);

        if (decelDistance > 0) {
            addMove(nominal_speed, leave_speed, -deceleration, decelDistance, block.axis_r, decelClocks);
        }
    }
    block.shaper_data.block_time = accelClocks + plateau / nominal_speed + decelClocks;

    block.shaper_data.move_end = prevMoveIndex(move_head);

    block.shaper_data.last_print_time = moves[block.shaper_data.move_end].end_t;
}

void MoveQueue::setMove(uint8_t move_index, float start_v, float end_v, float accelerate, float distance, xyze_float_t axis_r, float t, uint8_t flag) {
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
    move.start_t = last_move.end_t;
    move.end_t = move.start_t + move.t;

    for (int i = 0; i < AXIS_SIZE; ++i) {
        move.start_pos[i] = last_move.end_pos[i];
        move.end_pos[i] = last_move.end_pos[i] + move.distance * move.axis_r[i];
    }

    // LOG_I("move, %lf %lf %lf %lf %lf\n", move.start_t.toDouble(), move.end_t.toDouble(), move.accelerate, move.axis_r[0], move.distance);

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

uint8_t MoveQueue::addMove(float start_v, float end_v, float accelerate, float distance, xyze_float_t axis_r, float t, uint8_t flag) {
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
    uint8_t move_shaped_start = prevMoveIndex(index);
    float t = moves[move_shaped_start].t;

    while (t < delta_window && move_shaped_start != move_tail) {
        move_shaped_start = prevMoveIndex(move_shaped_start);
        t += moves[move_shaped_start].t;
    }
    return move_shaped_start;
};

uint8_t MoveQueue::calculateMoveEnd(uint8_t index, float delta_window) {
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