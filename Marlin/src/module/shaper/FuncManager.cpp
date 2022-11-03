#include "FuncManager.h"
#include "../AxisManager.h"
#include "../../../../snapmaker/debug/debug.h"

FuncParams FuncManager::FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
FuncParams FuncManager::FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];
FuncParams FuncManager::FUNC_PARAMS_T[FUNC_PARAMS_T_SIZE];

int8_t FuncManager::FUNC_PARAMS_TYPE_X[FUNC_PARAMS_X_SIZE];
int8_t FuncManager::FUNC_PARAMS_TYPE_Y[FUNC_PARAMS_Y_SIZE];
int8_t FuncManager::FUNC_PARAMS_TYPE_Z[FUNC_PARAMS_Z_SIZE];
int8_t FuncManager::FUNC_PARAMS_TYPE_E[FUNC_PARAMS_E_SIZE];
int8_t FuncManager::FUNC_PARAMS_TYPE_T[FUNC_PARAMS_T_SIZE];

float FuncManager::getPos(time_double_t time) {
    int func_start = func_params_tail;
    while (time > funcParams[func_start].right_time && func_start != prevFuncParamsIndex(func_params_head)) {
        func_start = nextFuncParamsIndex(func_start);
    }
    return getPosByFuncParams(time, func_start);
}

float FuncManager::getPosByFuncParams(time_double_t time, int func_params_use) {
    FuncParams& f_p = funcParams[func_params_use];
    time_double_t left_time = func_params_use == func_params_tail ? 0 : funcParams[prevFuncParamsIndex(func_params_use)].right_time;
    float t = time - left_time;
    return f_p.a * t * t + f_p.b * t + f_p.c;
}

FORCE_INLINE float FuncManager::getTimeByFuncParams(FuncParams* f_p, int8_t type, float pos, int func_params_use) {
    float a = f_p->a;
    float b = f_p->b;
    float c = f_p->c;
    c = c - pos;

    if (IS_ZERO(a)) {
        return -c / b;
    }

    float d2 = b * b - 4 * a * c;
    if (d2 < 0) {
        d2 = 0.0f;
    }

    float d = SQRT(d2);

    if (type > 0) {
        return (-b + d) / (2 * a);
    } else {
        return (-b - d) / (2 * a);
    }
}

void FuncManager::addFuncParams(float a, float b, float c, int type, time_double_t right_time, float right_pos) {
    if (max_size < getSize()) {
        max_size = getSize();
    }
    if (ABS(last_pos - right_pos) < EPSILON) {
        type = 0;
    }

    if (type == 0) {
        int func_params_use_tmp = func_params_use;
        if (last_is_zero && func_params_use_tmp != func_params_head && func_params_use_tmp != prevFuncParamsIndex(func_params_head)) {
            FuncParams &f_p = funcParams[prevFuncParamsIndex(func_params_head)];

            if (!IS_ZERO(f_p.right_pos - right_pos)) {
                LOG_I("error type: %lf, %lf\n", f_p.right_pos, right_pos);
            }

            f_p.right_time = right_time;
            f_p.right_pos = right_pos;

            last_time = right_time;
            last_pos = right_pos;

            return;
        } else {
            last_is_zero = true;
        }
    } else {
        last_is_zero = false;
    }


    FuncParams &f_p = funcParams[func_params_head];
    funcParamsTypes[func_params_head] = type;

    f_p.a = a;
    f_p.b = b;
    f_p.c = c;

    f_p.right_time = right_time;
    f_p.right_pos = right_pos;

    // if (type > 0 && right_pos <= last_pos) {
    //     LOG_I("error1 1 for right_pos: %lf, %lf\n", right_pos, last_pos);
    // }
    // if (type == 0 && right_pos != last_pos) {
    //     LOG_I("error1 0 for right_pos: %lf, %lf\n", right_pos, last_pos);
    // }
    // if (type < 0 && right_pos >= last_pos) {
    //     LOG_I("error1 -1 for right_pos: %lf, %lf\n", right_pos, last_pos);
    // }

    last_time = right_time;
    last_pos = right_pos;

    func_params_head = nextFuncParamsIndex(func_params_head);
}

bool FuncManager::getNextPosTime(int delta_step, int8_t *dir, float& mm_to_step, float& half_step_mm) {
    if (func_params_use == func_params_head) {
        return false;
    }

    if (average_index < average_count) {
        average_index++;
        print_step += average_step;
        print_time = average_print + average_delta_time * average_index;
        return true;
    }
    average_index = 0;
    average_count = 0;

    FuncParams *func_params = &funcParams[func_params_use];
    int8_t type = funcParamsTypes[func_params_use];

    int next_step = print_step;
    float next_pos = print_pos;
    while (func_params_use != func_params_head) {
        if (type == 0) {
        } else if (type > 0) {
            next_step = print_step + delta_step;
            next_pos = (float)next_step - 0.5f;
            if (next_pos <= func_params->right_pos + EPSILON) {
                *dir = 1;
                break;
            }
        } else {
            next_step = print_step - delta_step;
            next_pos = (float)next_step + 0.5f;
            if (next_pos >= func_params->right_pos - EPSILON) {
                *dir = -1;
                break;
            }
        }
        func_params_use = nextFuncParamsIndex(func_params_use);

        left_time = func_params->right_time;

        func_params = &funcParams[func_params_use];
        type = funcParamsTypes[func_params_use];
    }

    if (func_params_tail != func_params_use) {
        func_params_tail = prevFuncParamsIndex(func_params_use);
    }

    if (func_params_use == func_params_head) {
        return false;
    }

    time_double_t next_time = left_time + getTimeByFuncParams(func_params, type, next_pos, func_params_use);

    print_time = next_time;
    print_pos = next_pos;
    print_step = next_step;

    if (average_count == 0 && IS_ZERO(func_params->a)) {
        if (type > 0) {
            int count = FLOOR((func_params->right_pos + EPSILON - next_pos));
            if (count > 0) {
                average_count = count;
                average_step = delta_step;
                average_delta_time = (float) average_step / func_params->b;
                average_print = print_time;
            }
        } else if (type < 0) {
            int count = FLOOR((next_pos - func_params->right_pos + EPSILON));
            if (count > 0) {
                average_count = count;
                average_step = -delta_step;
                average_delta_time = (float) average_step / func_params->b;
                average_print = print_time;
            }
        }
    }

    return true;
}