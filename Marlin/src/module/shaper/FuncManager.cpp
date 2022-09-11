#include "FuncManager.h"
#include "../AxisManager.h"
#include "../../../../snapmaker/debug/debug.h"

FuncParams FuncManager::FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
FuncParams FuncManager::FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];

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

float FuncConsumer::getTimeByFuncParams(FuncParams* f_p, float pos, int func_params_use) {
    float a = f_p->a;
    float b = f_p->b;
    float c = f_p->c;
    c = c - pos;

    if (IS_ZERO(a)) {
        float k = -c / b;
        return k;
    }

    float d2 = b * b - 4 * a * c;
    if (d2 < 0) {
        d2 = 0.0f;
    }

    float d = SQRT(d2);

    if (f_p->type > 0) {
        float k = (-b + d) / (2 * a);
        return k;
    } else {
        float k = (-b - d) / (2 * a);
        return k;
    }
}

// void FuncManager::addMonotoneDeltaTimeFuncParams(float a, float b, float c, float delta_left_time, int8_t type, time_double_t right_time, float right_pos) {
//     // LOG_I("%lf, %lf, %lf, %lf, %d, %lf, %lf\n", a,b ,c, delta_left_time, type, right_time.toFloat(), right_pos);
//     if (max_size < getSize()) {
//         max_size = getSize();
//     }
//     // x = dx + left_time => f(x) = f(dx + left_time)
//     if (delta_left_time != 0) {
//         c = c + a * sq(delta_left_time) + b * delta_left_time;
//         b = b + 2 * a * delta_left_time;
//         a = a;
//     }

//     // if (axis < 2) {
//         // LOG_I("axis: %d, a: %lf, b: %lf, c: %lf, type: %d, x: %lf, y: %lf\n", axis, a,b,c,type, right_time.toDouble(), right_pos);
//     // }

//     if (a == 0 && b == 0) {
//         if (last_is_zero) {
//             FuncParams &f_p = funcParams[prevFuncParamsIndex(func_params_head)];

//             f_p.right_time = right_time;
//             f_p.right_pos = right_pos;

//             last_time = right_time;
//             last_pos = right_pos;

//             return;
//         } else {
//             last_is_zero = true;
//         }
//     } else {
//         last_is_zero = false;
//     }
    
//     FuncParams &f_p = funcParams[func_params_head];

//     f_p.a = a;
//     f_p.b = b;
//     f_p.c = c;
//     f_p.type = type;

//     f_p.right_time = right_time;
//     f_p.right_pos = right_pos;

//     last_time = right_time;
//     last_pos = right_pos;

//     func_params_head = nextFuncParamsIndex(func_params_head);
// }

// void FuncManager::addDeltaTimeFuncParams(float a, float b, float c, time_double_t  left_time, time_double_t right_time, float right_pos) {
//     float delta_left_time = 0;
//     float delta_right_time = right_time - left_time;

//     // LOG_I("add a: %lf, b: %lf, c: %lf, x: %lf, y: %lf\n", a,b,c, right_time.toDouble(), right_pos);

//     if (ABS(a) < EPSILON) {
//         a = 0;
//     }
//     if (ABS(b) < EPSILON) {
//         b = 0;
//     }

//     int8_t type = 0;
//     if (a == 0) {
//         type = b == 0 ? 0 : b > 0 ? 1 : -1;
//         addMonotoneDeltaTimeFuncParams(a, b, c, 0, type, right_time, right_pos);
//     } else {
//         float middle = -b / 2 / a;
//         if (delta_left_time < middle && delta_right_time > middle) {
//             type = a > 0 ? -1 : 1;
//             float middle_right_pos = getY(middle, a, b, c);
//             addMonotoneDeltaTimeFuncParams(a,b,c, 0, type, left_time + middle, middle_right_pos);
//             type = -type;
//             addMonotoneDeltaTimeFuncParams(a,b,c, middle, type, right_time, right_pos);
//         } else {
//             if (a > 0) {
//                 type = delta_left_time >= middle && delta_right_time >= middle ? 1 : -1;
//             } else {
//                 type = delta_left_time >= middle && delta_right_time >= middle ? -1 : 1;
//             }
//             addMonotoneDeltaTimeFuncParams(a,b,c, 0, type, right_time, right_pos);
//         }
//     }

//     last_time = right_time;
//     last_pos = right_pos;
// }

void FuncManager::addFuncParams(float a, float b, float c, int type, time_double_t right_time, float right_pos) {
    if (max_size < getSize()) {
        max_size = getSize();
    }
    if (ABS(last_pos - right_pos) < EPSILON) {
        type = 0;
    }

    if (type == 0) {
        int func_params_use = axisConsumerManager.axis_consumers[axis].func_consumer.func_params_use;
        if (last_is_zero && func_params_use != func_params_head && func_params_use != prevFuncParamsIndex(func_params_head)) {
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

    if (axis == 0)
    {
        // LOG_I("%d %lf %lf %lf %d %lf %lf\n",func_params_head, a, b, c, type, right_time.toFloat(), right_pos);
        // LOG_I("%d %lf %lf %lf\n",func_params_head, a, right_time.toFloat(), right_pos);
    }

    FuncParams &f_p = funcParams[func_params_head];

    f_p.a = a;
    f_p.b = b;
    f_p.c = c;
    f_p.type = type;

    f_p.right_time = right_time;
    f_p.right_pos = right_pos;

    if (type > 0 && right_pos <= last_pos) {
        LOG_I("error1 1 for right_pos: %lf, %lf\n", right_pos, last_pos);
    }
    if (type == 0 && right_pos != last_pos) {
        LOG_I("error1 0 for right_pos: %lf, %lf\n", right_pos, last_pos);
    }
    if (type < 0 && right_pos >= last_pos) {
        LOG_I("error1 -1 for right_pos: %lf, %lf\n", right_pos, last_pos);
    }

    last_time = right_time;
    last_pos = right_pos;

    func_params_head = nextFuncParamsIndex(func_params_head);
}

time_double_t* FuncConsumer::getNextPosTime(FuncParams* funcParams, int size, int func_params_head, int delta_step, int8_t *dir, float& mm_to_step, float& half_step_mm) {
    if (func_params_use == func_params_head) {
        return nullptr;
    }

    FuncParams *func_params = &funcParams[func_params_use];

    int next_step;
    float next_pos;
    while (func_params_use != func_params_head) {
        if (func_params->type == 0) {
        } else if (func_params->type > 0) {
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
        func_params_use = FuncManager::nextFuncParamsIndex(func_params_use, size);
        left_time = func_params->right_time;
        func_params = &funcParams[func_params_use];
    }

    if (func_params_use == func_params_head) {
        return nullptr;
    }

    // if (ABS(next_pos - func_params->right_pos) < EPSILON) {
    //     print_time = func_params->right_time;
    //     print_pos = func_params->right_pos;
    //     print_step = next_step;
    //     return &print_time;
    // }

    time_double_t next_time;
    if (func_params->type == 0) {
        next_time = func_params->right_time;
    } else {
        next_time = left_time + getTimeByFuncParams(func_params, next_pos, func_params_use);
    }

    print_time = next_time;
    print_pos = next_pos;
    print_step = next_step;

    return &print_time;
}