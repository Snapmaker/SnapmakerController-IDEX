#pragma once

#include <cstdint>
#include "TimeDouble.h"
#include "../../MarlinCore.h"
#include "../../../../snapmaker/debug/debug.h"

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
    int axis;
    int size;
    int max_size = 0;
    time_double_t last_time = 0;
    float last_pos = 0;
    bool last_is_zero = false;

    // Consume
    time_double_t print_time = 0;
    time_double_t left_time = 0;
    float print_pos = 0;
    int print_step = 0;
    

    static FuncParams FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
    static FuncParams FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
    static FuncParams FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
    static FuncParams FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];

    FuncParams* funcParams;
    volatile int func_params_tail = 0;
    volatile int func_params_use = 0;
    volatile int func_params_head = 0;

    FuncManager(){};

    void init(int8_t axis) {
        this->axis = axis;
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

    void abort() {
        // LOG_I("abort\n");
        func_params_tail = func_params_head = 0;
        last_time = 0;
        last_pos = 0;
        last_is_zero = false;

        print_time = 0;
        left_time = 0;
        print_pos = 0;
        print_step = 0;
        func_params_use = 0;
    }

    constexpr int getSize() {
        return FUNC_PARAMS_MOD(func_params_head - func_params_tail, size);
    }

    constexpr int nextFuncParamsIndex(const int func_params_index) { return FUNC_PARAMS_MOD(func_params_index + 1, size); };
    constexpr int prevFuncParamsIndex(const int func_params_index) { return FUNC_PARAMS_MOD(func_params_index - 1, size); };
    // static constexpr int nextFuncParamsIndex(const int func_params_index, int s) { return FUNC_PARAMS_MOD(func_params_index + 1, s); };
    // static constexpr int prevFuncParamsIndex(const int func_params_index, int s) { return FUNC_PARAMS_MOD(func_params_index - 1, s); };

    constexpr bool isBetween(const int func_params_start, const int func_params_end, const int func_params_middle) {
        return (FUNC_PARAMS_MOD(func_params_middle - func_params_start, size) + FUNC_PARAMS_MOD(func_params_end - func_params_middle, size)) == FUNC_PARAMS_MOD(func_params_end - func_params_start, size);
    };

    // void addMonotoneDeltaTimeFuncParams(float a, float b, float c, float delta_left_time, int8_t type, time_double_t right_time, float right_pos);

    // void addDeltaTimeFuncParams(float a, float b, float c, time_double_t left_time, time_double_t right_time, float right_pos);
    
    void addFuncParams(float a, float b, float c,int type, time_double_t right_time, float right_pos);

    float getPos(time_double_t time);

    float getY(float x, float a, float b, float c) {
        return a * sq(x) + b * x + c;
    };

    //    float getXAndMove(float y, int *func_params_start, int func_params_end);

    time_double_t *getNextPosTime(int delta_step, int8_t *dir, float& mm_to_step, float& half_step_mm);

  private:

    float getPosByFuncParams(time_double_t time, int func_params_use);

    float getTimeByFuncParams(FuncParams* f_p, float pos, int func_params_use);
};