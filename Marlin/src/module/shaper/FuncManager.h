#pragma once

#include <cstdint>
#include "TimeDouble.h"
#include "../../MarlinCore.h"

#define FUNC_PARAMS_SIZE 512
#define FUNC_PARAMS_MOD(n, size) ((n + size) % size)

class FuncParams {
  public:
    float a, b, c;
    time_double_t right_time = 0;
    float right_pos = 0;
    int8_t type = 0;

    void update(float a, float b, float c) {
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

#define FUNC_PARAMS_X_SIZE 128
#define FUNC_PARAMS_Y_SIZE 128
#define FUNC_PARAMS_Z_SIZE 64
#define FUNC_PARAMS_E_SIZE 64

// static FuncParams FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
// static FuncParams FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
// static FuncParams FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
// static FuncParams FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];

class FuncManager {
  public:
    int size;
    time_double_t print_time = 0;
    time_double_t last_time = 0;
    float print_pos = 0;
    float last_pos = 0;

    static FuncParams FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
    static FuncParams FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
    static FuncParams FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
    static FuncParams FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];

    FuncParams* funcParams;
    volatile uint32_t func_params_tail = 0;
    volatile uint32_t func_params_use = 0;
    volatile uint32_t func_params_head = 0;

    FuncManager(){};

    void init(int8_t axis) {
        switch (axis) {
            case 0:
                size = FUNC_PARAMS_X_SIZE;
                funcParams = FUNC_PARAMS_X;
                break;
            case 1:
                size = FUNC_PARAMS_Y_SIZE;
                funcParams = FUNC_PARAMS_Y;
                break;
            case 2:
                size = FUNC_PARAMS_Z_SIZE;
                funcParams = FUNC_PARAMS_Z;
                break;
            case 3:
                size = FUNC_PARAMS_E_SIZE;
                funcParams = FUNC_PARAMS_E;
                break;
        }
    }

    constexpr uint32_t nextFuncParamsIndex(const uint32_t func_params_index) { return FUNC_PARAMS_MOD(func_params_index + 1, size); };
    constexpr uint32_t prevFuncParamsIndex(const uint32_t func_params_index) { return FUNC_PARAMS_MOD(func_params_index - 1, size); };

    constexpr bool isBetween(const uint32_t func_params_start, const uint32_t func_params_end, const uint32_t func_params_middle) {
        return (FUNC_PARAMS_MOD(func_params_middle - func_params_start, size) + FUNC_PARAMS_MOD(func_params_end - func_params_middle, size)) == FUNC_PARAMS_MOD(func_params_end - func_params_start, size);
    };

    void addMonotoneDeltaTimeFuncParams(float a, float b, float c, float delta_left_time, int8_t type, time_double_t right_time, float right_pos);

    void addDeltaTimeFuncParams(float a, float b, float c, time_double_t left_time, time_double_t right_time, float right_pos);

    float getPos(time_double_t time);

    time_double_t *getNextPosTime(float delta_pos, int8_t *dir);

    float getY(float x, float a, float b, float c) {
        return a * sq(x) + b * x + c;
    };

    //    float getXAndMove(float y, uint32_t *func_params_start, uint32_t func_params_end);

  private:
    time_double_t getTimeByFuncParams(float pos, uint32_t func_params_use);

    float getPosByFuncParams(time_double_t time, uint32_t func_params_use);
};