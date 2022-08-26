#include "FuncManager.h"

FuncParams FuncManager::FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
FuncParams FuncManager::FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];

float FuncManager::getPos(time_double_t time) {
    uint32_t func_start = func_params_tail;
    while (time > funcParams[func_start].right_time && func_start != func_params_head) {
        func_start = nextFuncParamsIndex(func_start);
    }
    return getPosByFuncParams(time, func_start);
}

float FuncManager::getPosByFuncParams(time_double_t time, uint32_t func_params_use) {
    FuncParams& f_p = funcParams[func_params_use];
    time_double_t left_time = func_params_use == func_params_tail ? 0 : funcParams[prevFuncParamsIndex(func_params_use)].right_time;
    float t = time - left_time;
    return f_p.a * t * t + f_p.b * t + f_p.c;
}

time_double_t FuncManager::getTimeByFuncParams(float pos, uint32_t func_params_use) {
    FuncParams& f_p = funcParams[func_params_use];

    time_double_t left_time = func_params_use == func_params_tail ? 0 : funcParams[prevFuncParamsIndex(func_params_use)].right_time;

    float a = f_p.a;
    float b = f_p.b;
    float c = f_p.c;
    c = c - pos;

    if (a == 0) {
        return left_time + -c / b;
    }

    float d2 = b * b - 4 * a * c;
    if (d2 < 0) {
        return 0;
    }

    float d = SQRT(d2);

    if (f_p.type > 0) {
        return  left_time + (-b + d) / (2 * a);
    } else {
        return left_time + (-b - d) / (2 * a);
    }
}

void FuncManager::addMonotoneDeltaTimeFuncParams(float a, float b, float c, float delta_left_time, int8_t type, time_double_t right_time, float right_pos) {
    FuncParams &f_p = funcParams[func_params_head];

    // x = dx + left_time => f(x) = f(dx + left_time)
    if (delta_left_time != 0) {
        c = c + a * sq(delta_left_time) + b * delta_left_time;
        b = b + 2 * a * delta_left_time;
        a = a;
    }

    f_p.a = a;
    f_p.b = b;
    f_p.c = c;
    f_p.type = type;

    f_p.right_time = right_time;
    f_p.right_pos = right_pos;

    func_params_head = nextFuncParamsIndex(func_params_head);
}

void FuncManager::addDeltaTimeFuncParams(float a, float b, float c, time_double_t  left_time, time_double_t right_time, float right_pos) {
    float delta_left_time = 0;
    float delta_right_time = right_time - left_time;

    if (ABS(a) < EPSILON) {
        a = 0;
    }
    if (ABS(b) < EPSILON) {
        b = 0;
    }

    int8_t type = 0;
    if (a == 0) {
        type = b == 0 ? 0 : b > 0 ? 1 : -1;
        addMonotoneDeltaTimeFuncParams(a, b, c, 0, type, right_time, right_pos);
    } else {
        float middle = -b / 2 / a;
        if (delta_left_time < middle && delta_right_time > middle) {
            type = a > 0 ? -1 : 1;
            float middle_right_pos = getY(middle, a, b, c);
            addMonotoneDeltaTimeFuncParams(a,b,c, 0, type, left_time + middle, middle_right_pos);
            type = -type;
            addMonotoneDeltaTimeFuncParams(a,b,c, middle, type, right_time, right_pos);
        } else {
            if (a > 0) {
                type = delta_left_time >= middle && delta_right_time >= middle ? 1 : -1;
            } else {
                type = delta_left_time >= middle && delta_right_time >= middle ? -1 : 1;
            }
            addMonotoneDeltaTimeFuncParams(a,b,c, 0, type, right_time, right_pos);
        }
    }

    last_time = right_time;
    last_pos = right_pos;
}

time_double_t* FuncManager::getNextPosTime(float delta_pos,int8_t* dir) {
    if (print_time == last_time) {
        return nullptr;
    }

    FuncParams *func_params = &funcParams[func_params_use];

    int prev_type = func_params->type;
    float next_pos;
    bool func_params_move = false;
    while (func_params_use != func_params_head) {
        if (func_params->type == 0) {
        } else if (func_params->type > 0) {
            if (prev_type == -1) {
                next_pos = func_params_move ? print_pos : print_pos + delta_pos;
            } else {
                next_pos = print_pos + delta_pos;
            }
            if (next_pos <= func_params->right_pos + EPSILON) {
                *dir = 1;
                break;
            }
        } else {
            if (prev_type == 1) {
                next_pos = func_params_move ? print_pos : print_pos - delta_pos;
            } else {
                next_pos = print_pos - delta_pos;
            }
            if (next_pos >= func_params->right_pos - EPSILON) {
                *dir = -1;
                break;
            }
        }
        func_params_move = true;
        func_params_use = nextFuncParamsIndex(func_params_use);
        func_params = &funcParams[func_params_use];
    }

    if (func_params_use == func_params_head) {
        return nullptr;
    }

    if (ABS(next_pos - func_params->right_pos) < EPSILON) {
        print_time = func_params->right_time;
        print_pos = func_params->right_pos;
        return &print_time;
    }

    time_double_t next_time;
    if (func_params->type == 0) {
        next_time = func_params->right_time;
    } else {
        next_time = getTimeByFuncParams(next_pos, func_params_use);
    }

    if (func_params_tail != func_params_use) {
        func_params_tail = prevFuncParamsIndex(func_params_use);
    }

    print_time = next_time;
    print_pos = next_pos;

    return &print_time;
}