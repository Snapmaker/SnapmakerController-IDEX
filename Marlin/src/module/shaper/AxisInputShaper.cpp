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

#include "AxisInputShaper.h"
#include "MoveQueue.h"
#include "../../../../snapmaker/debug/debug.h"

AxisInputShaper AxisInputShaper::axis_input_shaper_x;
AxisInputShaper AxisInputShaper::axis_input_shaper_y;

void AxisInputShaper::init()
{
    params.n = 0;
    is_shaper_window_init = false;
    switch (type) {
        case InputShaperType::none: {
            params.n = 1;
            params.A[0] = 1;
            params.T[0] = 0;
            break;
        }

        case InputShaperType::zv: {
            float df = SQRT(1. - sq(zeta));
            float K = expf(-zeta * M_PI / df);
            float t_d = 1. / (frequency * df);
            params.n = 2;
            params.A[0] = 1;
            params.A[1] = K;
            params.T[0] = 0;
            params.T[1] = 0.5 * t_d;
            break;
        }

        case InputShaperType::zvd: {
            float df = SQRT(1. - sq(zeta));
            float K = expf(-zeta * M_PI / df);
            float t_d = 1. / (frequency * df);
            params.n = 3;
            params.A[0] = 1;
            params.A[1] = 2 * K;
            params.A[2] = sq(K);
            params.T[0] = 0;
            params.T[1] = 0.5 * t_d;
            params.T[2] = t_d;
            break;
        }

        case InputShaperType::zvdd: {
            float df = SQRT(1. - sq(zeta));
            float K = expf(-zeta * M_PI / df);
            float t_d = 1. / (frequency * df);
            params.n = 4;

            params.A[0] = 1;
            params.A[1] = 3 * K;
            params.A[2] = 3 * sq(K);
            params.A[3] = K * K * K;

            params.T[0] = 0;
            params.T[1] = 0.5 * t_d;
            params.T[2] = t_d;
            params.T[3] = 1.5 * t_d;
            break;
        }

        case InputShaperType::zvddd: {
            float df = SQRT(1. - sq(zeta));
            float K = expf(-zeta * M_PI / df);
            float t_d = 1. / (frequency * df);
            params.n = 4;

            params.A[0] = 1;
            params.A[1] = 4 * K;
            params.A[2] = 6 * sq(K);
            params.A[3] = 4 * K * K * K;
            params.A[4] = K * K * K * K;

            params.T[0] = 0;
            params.T[1] = 0.5 * t_d;
            params.T[2] = t_d;
            params.T[3] = 1.5 * t_d;
            params.T[4] = 2 * t_d;
            break;
        }

        case InputShaperType::mzv: {
            float df = SQRT(1. - sq(zeta));
            float K = expf(-.75 * zeta * M_PI / df);
            float t_d = 1. / (frequency * df);

            float a1 = 1. - 1. / SQRT(2.);
            float a2 = (SQRT(2.) - 1.) * K;
            float a3 = a1 * K * K;

            params.n = 3;
            params.A[0] = a1;
            params.A[1] = a2;
            params.A[2] = a3;

            params.T[0] = 0.;
            params.T[1] = .375*t_d;
            params.T[2] = .75*t_d;
            break;
        }

        case InputShaperType::ei: {
            float v_tol = 1. / SHAPER_VIBRATION_REDUCTION; // vibration tolerance
            float df = SQRT(1. - sq(zeta));
            float K = expf(-zeta * M_PI / df);
            float t_d = 1. / (frequency * df);

            float a1 = .25 * (1. + v_tol);
            float a2 = .5 * (1. - v_tol) * K;
            float a3 = a1 * K * K;

            params.n = 3;
            params.A[0] = a1;
            params.A[1] = a2;
            params.A[2] = a3;

            params.T[0] = 0;
            params.T[1] = 0.5 * t_d;
            params.T[2] = t_d;
            break;
        }

        case InputShaperType::ei2: {
            float v_tol = 1. / SHAPER_VIBRATION_REDUCTION; // vibration tolerance
            float df = SQRT(1. - sq(zeta));
            float K = expf(-zeta * M_PI / df);
            float t_d = 1. / (frequency * df);

            float V2 = sq(v_tol);
            float X = pow(V2 * (SQRT(1. - V2) + 1.), 1./3.);
            float a1 = (3.*X*X + 2.*X + 3.*V2) / (16.*X);
            float a2 = (.5 - a1) * K;
            float a3 = a2 * K;
            float a4 = a1 * K * K * K;

            params.n = 4;
            params.A[0] = a1;
            params.A[1] = a2;
            params.A[2] = a3;
            params.A[3] = a4;

            params.T[0] = 0;
            params.T[1] = 0.5 * t_d;
            params.T[2] = t_d;
            params.T[3] = 1.5 * t_d;
            break;
        }

        case InputShaperType::ei3: {
            float v_tol = 1. / SHAPER_VIBRATION_REDUCTION;
            float df = SQRT(1. - sq(zeta));
            float K = expf(-zeta * M_PI / df);
            float t_d = 1. / (frequency * df);

            float K2 = K * K;
            float a1 = 0.0625 * (1. + 3. * v_tol + 2. * SQRT(2. * (v_tol + 1.) * v_tol));
            float a2 = 0.25 * (1. - v_tol) * K;
            float a3 = (0.5 * (1. + v_tol) - 2. * a1) * K2;
            float a4 = a2 * K2;
            float a5 = a1 * K2 * K2;

            params.n = 5;
            params.A[0] = a1;
            params.A[1] = a2;
            params.A[2] = a3;
            params.A[3] = a4;
            params.A[4] = a5;

            params.T[0] = 0;
            params.T[1] = 0.5 * t_d;
            params.T[2] = t_d;
            params.T[3] = 1.5 * t_d;
            params.T[4] = 2 * t_d;
            break;
        }

        default: {
            params.n = 1;
            params.A[0] = 1;
            params.T[0] = 0;
            break;
        }
    }

    shiftPulses();

    logParams();
}

void AxisInputShaper::logParams() {
    // LOG_I("logParams\n");
    // LOG_I("n: %d\n", shift_params.n);
    // for (int i = 0; i < shift_params.n; i++)
    // {
    //     LOG_I("i: %d, A: %lf, T: %lf\n", i, shift_params.A[i], shift_params.T[i]);
    // }
}

void AxisInputShaper::shiftPulses() {
    float sum_a = 0.;
    for (int i = 0; i < params.n; ++i)
        sum_a += params.A[i];
    float inv_a = 1. / sum_a;
    for (int i = 0; i < params.n; ++i) {
        shift_params.A[params.n - i - 1] = params.A[i] * inv_a;
        shift_params.T[params.n - i - 1] = -params.T[i];
    }
    shift_params.n = params.n;

    sum_a = 0;
    for (int i = 0; i < params.n; ++i) {
        shift_params.T[i] = shift_params.T[i] * 1000;
    }

    float ts = 0.;
    for (int i = 0; i < shift_params.n; ++i)
        ts += shift_params.A[i] * shift_params.T[i];
    for (int i = 0; i < shift_params.n; ++i)
        shift_params.T[i] -= ts;

    left_delta = params.n == 0 ? 0 : ABS(shift_params.T[0]);
    right_delta = params.n == 0 ? 0 : ABS(shift_params.T[shift_params.n - 1]);
    delta_window = right_delta + left_delta;
}

FORCE_INLINE void AxisInputShaper::addFuncParamsToManager(FuncManager *func_manager,float a, time_double_t right_time, float right_pos, float x2, float y1, float y2) {
    float b = 0, c = 0;

    float dx = x2;
    float dy = y2 - y1;
    if (dx < EPSILON) {
        return;
    }

    if (IS_ZERO(a)) {
        b = dy / dx;
        c = y1;

        int type = IS_ZERO(b) ? 0 : b > 0 ? 1 : -1;
        func_manager->addFuncParams(a, b, c, type, right_time, right_pos);
    } else {
        b = dy / dx - a * x2;
        c = y1;
        float middle = 0.5f * x2 - dy / (dx * 2 * a);

        if (EPSILON < middle && middle < x2 - EPSILON) {
            float k = dy / dx;
            float middle_pos = c - 0.25f * (sq(k) / a - 2 * k * x2 + a * sq(x2));
            int type = a > 0 ? -1 : 1;
            time_double_t middle_time = func_manager->last_time + middle;
            func_manager->addFuncParams(a, b, c, type, middle_time, middle_pos);

            type = -type;
            b = 0.0f;
            c = middle_pos;
            func_manager->addFuncParams(a, b, c, type, right_time, right_pos);
        } else {
            int type = a > 0 ? middle < EPSILON ? 1 : -1 :middle < EPSILON ? -1 : 1;
            func_manager->addFuncParams(a, b, c, type, right_time, right_pos);
        }
    }
}

float AxisInputShaper::calcPosition(int move_index, time_double_t time, int move_shaped_start, int move_shaped_end) {
    if (moveQueue.getMoveSize() == 0) {
        LOG_I("moveQueue.getMoveSize() zero\n");
        return 0;
    }

    if (!moveQueue.isBetween(move_index)) {
        LOG_I("isb %d, %d, %d\n",move_index, moveQueue.move_tail, moveQueue.move_head);
        return 0;
    }

    if (move_index == moveQueue.move_head) {
        LOG_I("move_index == head\n");
    }

    float res = 0;
    for (int i = 0; i < shift_params.n; i++) {
        time_double_t t = time + shift_params.T[i];
        res += shift_params.A[i] * moveQueue.getAxisPositionAcrossMoves(move_index, axis, t, move_shaped_start, move_shaped_end);
    }
    return res;
}


FORCE_INLINE void AxisInputShaper::moveShaperWindowByIndex(FuncManager *func_manager, int move_shaped_start, int move_shaped_end) {
    int n = shift_params.n;
    shaper_window.n = n;
    shaper_window.zero_n = n - 1;

    Move *move = &moveQueue.moves[move_shaped_start];

    shaper_window.time = move->end_t - right_delta;

    for (int i = n - 1; i >= 0; i--) {
        ShaperWindowParams &w_p = shaper_window.params[i];
        w_p.A = shift_params.A[i];
        w_p.T = shift_params.T[i];

        w_p.time = shaper_window.time + w_p.T;

        while (w_p.time <= move->start_t && move_shaped_start != moveQueue.move_tail) {
            move_shaped_start = moveQueue.prevMoveIndex(move_shaped_start);
            move = &moveQueue.moves[move_shaped_start];
        }

        w_p.move_index = move_shaped_start;
        w_p.a = 0.5f * move->accelerate * w_p.A * move->axis_r[axis];
        // shaper_window.updateParamABC(i, move->start_v, move->accelerate, move->start_t, left_time, move->start_pos[axis], move->axis_r[axis]);
    }

    time_double_t shaper_time = shaper_window.time;
    shaper_window.pos = calcPosition(move_shaped_start, shaper_time, move_shaped_start, move_shaped_end);

    // if (shaper_window.pos < -48000 || shaper_window.pos > 48000) {
    //     LOG_I("debug\n");
    //     shaper_window.pos = calcPosition(move_shaped_start, shaper_window.time, move_shaped_start, move_shaped_end);
    // }

    float y1 = func_manager->last_pos;
    float x2 = shaper_window.time - func_manager->last_time;
    float y2 = shaper_window.pos;

    // shaper_window.updateABC();
    // shaper_window.updateABC(x1, y1, x2, y2);
    shaper_window.updateParamsA();
    addFuncParamsToManager(func_manager, shaper_window.func_params.a, shaper_time, y2, x2, y1, y2);

    // LOG_I("move_index: %d, %d\n", shaper_window.params[0].move_index, shaper_window.params[1].move_index);
}


bool AxisInputShaper::moveShaperWindowToNext(FuncManager *func_manager, uint8_t move_shaped_start, uint8_t move_shaped_end) {
    ShaperWindowParams *zero_p = &shaper_window.params[shaper_window.zero_n];

    if (zero_p->move_index == move_shaped_end)
    {
        return false;
    }


    zero_p->move_index = moveQueue.nextMoveIndex(zero_p->move_index);

    Move *move = &moveQueue.moves[zero_p->move_index];

    ShaperWindowParams& w_p = shaper_window.params[shaper_window.zero_n];
    w_p.a = 0.5f * move->accelerate * w_p.A * move->axis_r[axis];

    // shaper_window.updateParamABC(shaper_window.zero_n, move->start_v, move->accelerate, move->start_t, left_time, move->start_pos[axis], move->axis_r[axis]);

    float min_next_time = 1000000000.0f;

    for (int i = 0; i < shaper_window.n; i++) {
        ShaperWindowParams &p = shaper_window.params[i];

        float min_p_next_time = moveQueue.moves[p.move_index].end_t - p.time;

        if (min_p_next_time < min_next_time) {
            min_next_time = min_p_next_time;
            shaper_window.zero_n = i;
        }
    }

    // Cumulative error of processing value
    zero_p = &shaper_window.params[shaper_window.zero_n];
    zero_p->time = moveQueue.moves[zero_p->move_index].end_t;

    for (int i = 0; i < shaper_window.n; i++) {
        if (i == shaper_window.zero_n) {
            continue;
        }
        ShaperWindowParams &p = shaper_window.params[i];
        p.time += min_next_time;
    }

    // shaper_window.updateParamLeftTime(left_time);
    time_double_t new_time = shaper_window.time + min_next_time;
    shaper_window.time = new_time;
    int move_index = shaper_window.params[0].move_index;
    shaper_window.pos = calcPosition(move_index, shaper_window.time, move_index, move_shaped_end);

    // if (shaper_window.pos < -48000 || shaper_window.pos > 48000) {
    //     LOG_I("debug\n");
    //     shaper_window.pos = calcPosition(move_shaped_start, shaper_window.time, move_shaped_start, move_shaped_end);
    // }

    float y1 = func_manager->last_pos;
    float x2 = shaper_window.time - func_manager->last_time;
    float y2 = shaper_window.pos;

    shaper_window.updateParamsA();
    addFuncParamsToManager(func_manager, shaper_window.func_params.a, new_time, y2, x2, y1, y2);
    // shaper_window.updateABC();
    // shaper_window.updateABC(x1, y1, x2, y2);


    // LOG_I("a:%lf %lf, b: %lf %lf, c: %lf %lf\n",
        // shaper_window.func_params.a, shaper_window.new_func_params.a,
        // shaper_window.func_params.b, shaper_window.new_func_params.b,
        // shaper_window.func_params.c, shaper_window.new_func_params.c
        // );

    return true;
}

bool AxisInputShaper::generateShapedFuncParams(FuncManager* func_manager, uint8_t move_shaper_start, uint8_t move_shaper_end) {
    if (!is_shaper_window_init) {
        moveShaperWindowByIndex(func_manager, move_shaper_start, move_shaper_end);
        // func_manager.addDeltaTimeFuncParams(shaper_window.func_params.a, shaper_window.func_params.b, shaper_window.func_params.c, func_manager.last_time, shaper_window.time, shaper_window.pos);

        is_shaper_window_init = true;
    }

    while (moveShaperWindowToNext(func_manager, move_shaper_start, move_shaper_end)) {
        // func_manager.addDeltaTimeFuncParams(shaper_window.func_params.a, shaper_window.func_params.b, shaper_window.func_params.c, func_manager.last_time, shaper_window.time, shaper_window.pos);
        // printf("axis: %d, generateFuncParams: %lf, %lf, %lf \n", axis, shaper_window.pos, func_manager.getPos(shaper_window.time), shaper_window.time);
    }

    // if (func_manager.max_size < func_manager.getSize())
    // {
        // func_manager.max_size = func_manager.getSize();
    // }


    return true;
}
