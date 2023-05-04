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

#pragma once

#include <cstdint>
#include "TimeDouble.h"
#include "../../MarlinCore.h"
#include "../../../../snapmaker/debug/debug.h"

#define E_START_POS     (0.0)
// #define E_START_POS     ((16.0 * 4157 * 138))
#define FUNC_PARAMS_SIZE 512
#define FUNC_PARAMS_MOD(n, size) ((n + size) % size)

class FuncParams {
  public:
    float a, b, c, right_pos;
    time_double_t right_time = 0;

    void update(float a, float b, float c) {
        this->a = a;
        this->b = b;
        this->c = c;
    }
};

class FuncParamsExtend {
  public:
    double a, b, c, right_pos;
    time_double_t right_time = 0;

    void update(double a, double b, double c) {
        this->a = a;
        this->b = b;
        this->c = c;
    }
};

//class Func {
//  public:
//    float a, b, c;
//
//    Func(){};
//    Func(float a, float b, float c) : a(a), b(b), c(c){};
//
//    void update(float a, float b, float c);
//
//    float getY(float x);
//
//    static float getY(float x, float a, float b, float c);
//    static float getX(float y, float a, float b, float c, float left_time, int8_t type);
//};

#define FUNC_PARAMS_X_SIZE 300
#define FUNC_PARAMS_Y_SIZE 300
#define FUNC_PARAMS_Z_SIZE 64
#define FUNC_PARAMS_E_SIZE 64
#define FUNC_PARAMS_T_SIZE 8

// static FuncParams FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
// static FuncParams FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
// static FuncParams FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
// static FuncParams FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];

class FuncManager {
  private:
    // Fast calculation parameters of average speed section
    int average_index = 0;
    int average_count = 0;
    int average_step = 0;
    float average_delta_time = 0;
    time_double_t average_print = 0;

  public:
    static FuncParams FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
    static FuncParams FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
    static FuncParams FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
    static FuncParamsExtend FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];
    static FuncParams FUNC_PARAMS_T[FUNC_PARAMS_T_SIZE];

    static int8_t FUNC_PARAMS_TYPE_X[FUNC_PARAMS_X_SIZE];
    static int8_t FUNC_PARAMS_TYPE_Y[FUNC_PARAMS_Y_SIZE];
    static int8_t FUNC_PARAMS_TYPE_Z[FUNC_PARAMS_Z_SIZE];
    static int8_t FUNC_PARAMS_TYPE_E[FUNC_PARAMS_E_SIZE];
    static int8_t FUNC_PARAMS_TYPE_T[FUNC_PARAMS_T_SIZE];

    FuncParams* funcParams;
    FuncParamsExtend *funcParamsExtend;
    int8_t* funcParamsTypes;

    volatile int func_params_tail = 0;
    volatile int func_params_use = 0;
    volatile int func_params_head = 0;

    // Axis index
    int axis;
    // Piecewise function length
    int size;
    // Short time maximum length of piecewise function
    int max_size = 0;

    time_double_t last_time = 0;
    float last_pos = 0;
    double last_pos_e = 0;
    bool last_is_zero = false;

    // Consume
    time_double_t left_time = 0;
    time_double_t print_time = 0;
    float print_pos = 0;
    double print_pos_e = 0;
    int print_step = 0;

    FuncManager(){};

    void init(int8_t axis) {
        this->axis = axis;
        switch (axis) {
            case 0:
                size = FUNC_PARAMS_X_SIZE;
                funcParams = FUNC_PARAMS_X;
                funcParamsTypes = FUNC_PARAMS_TYPE_X;
                break;
            case 1:
                size = FUNC_PARAMS_Y_SIZE;
                funcParams = FUNC_PARAMS_Y;
                funcParamsTypes = FUNC_PARAMS_TYPE_Y;
                break;
            case 2:
                size = FUNC_PARAMS_Z_SIZE;
                funcParams = FUNC_PARAMS_Z;
                funcParamsTypes = FUNC_PARAMS_TYPE_Z;
                break;
            case 3:
                size = FUNC_PARAMS_E_SIZE;
                funcParamsExtend = FUNC_PARAMS_E;
                funcParamsTypes = FUNC_PARAMS_TYPE_E;
                break;
            case 4:
                size = FUNC_PARAMS_T_SIZE;
                funcParams = FUNC_PARAMS_T;
                funcParamsTypes = FUNC_PARAMS_TYPE_T;
                break;
        }
    }

    void reset() {
        func_params_tail = 0;
        func_params_use = 0;
        func_params_head = 0;

        last_time = 0;
        last_pos = 0;
        last_pos_e = E_START_POS;
        last_is_zero = false;

        left_time = 0;
        print_time = 0;
        print_pos = 0;
        print_pos_e = E_START_POS;
        if (E_AXIS == axis)
          print_step = E_START_POS;
        else
          print_step = 0;

        average_index = 0;
        average_count = 0;
        average_step = 0;
        average_delta_time = 0;
        average_print = 0;
    }

    constexpr int getSize() {
      return FUNC_PARAMS_MOD(func_params_head - func_params_tail, size);
    }

    constexpr int getFreeSize() {
        return size - 1 - getSize();
    }

    FORCE_INLINE constexpr int nextFuncParamsIndex(const int func_params_index) { return FUNC_PARAMS_MOD(func_params_index + 1, size); };
    FORCE_INLINE constexpr int prevFuncParamsIndex(const int func_params_index) { return FUNC_PARAMS_MOD(func_params_index - 1, size); };
    // static constexpr int nextFuncParamsIndex(const int func_params_index, int s) { return FUNC_PARAMS_MOD(func_params_index + 1, s); };
    // static constexpr int prevFuncParamsIndex(const int func_params_index, int s) { return FUNC_PARAMS_MOD(func_params_index - 1, s); };

    constexpr bool isBetween(const int func_params_start, const int func_params_end, const int func_params_middle) {
        return (FUNC_PARAMS_MOD(func_params_middle - func_params_start, size) + FUNC_PARAMS_MOD(func_params_end - func_params_middle, size)) == FUNC_PARAMS_MOD(func_params_end - func_params_start, size);
    };

    // void addMonotoneDeltaTimeFuncParams(float a, float b, float c, float delta_left_time, int8_t type, time_double_t right_time, float right_pos);

    // void addDeltaTimeFuncParams(float a, float b, float c, time_double_t left_time, time_double_t right_time, float right_pos);

    void addFuncParams(float a, float b, float c,int type, time_double_t right_time, float right_pos);
    void addFuncParamsExtend(double a, double b, double c, int type, time_double_t right_time, double right_pos);

    float getPos(time_double_t time);

    float getY(float x, float a, float b, float c) {
        return a * sq(x) + b * x + c;
    };

    //    float getXAndMove(float y, int *func_params_start, int func_params_end);

    bool getNextPosTime(int delta_step, int8_t *dir, float& mm_to_step, float& half_step_mm);
    bool getNextPosTimeEextend(int delta_step, int8_t *dir, float& mm_to_step, float& half_step_mm);

  private:

    float getPosByFuncParams(time_double_t time, int func_params_use);

    FORCE_INLINE float getTimeByFuncParams(FuncParams* f_p, int8_t type, float pos, int func_params_use);
    FORCE_INLINE double getTimeByFuncParamsExtend(FuncParamsExtend* f_p, int8_t type, double pos, int func_params_use);
};