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

#include "FuncManager.h"
#include "../AxisManager.h"
#include "../../../../snapmaker/debug/debug.h"

FuncParams FuncManager::FUNC_PARAMS_X[FUNC_PARAMS_X_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Y[FUNC_PARAMS_Y_SIZE];
FuncParams FuncManager::FUNC_PARAMS_Z[FUNC_PARAMS_Z_SIZE];
FuncParamsExtend FuncManager::FUNC_PARAMS_E[FUNC_PARAMS_E_SIZE];
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

FORCE_INLINE double FuncManager::getTimeByFuncParamsExtend(FuncParamsExtend* f_p, int8_t type, double pos, int func_params_use) {
    double a = f_p->a;
    double b = f_p->b;
    double c = f_p->c;
    c = c - pos;

    if (IS_ZERO(a)) {
        return -c / b;
    }

    double d2 = b * b - 4 * a * c;
    if (d2 < 0) {
        d2 = 0.0f;
    }

    double d = SQRT(d2);

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
                LOG_I("error type: %lf, %lf, a: %d\n", f_p.right_pos, right_pos, axis);
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

    if (!last_is_zero && func_params_head == func_params_use) {
      extern uint32_t statistics_funcgen_runout_cnt;
      statistics_funcgen_runout_cnt++;
      LOG_E("statistics_funcgen_runout_cnt on axi %d\r\n", axis);
    }
}

void FuncManager::addFuncParamsExtend(double a, double b, double c, int type, time_double_t right_time, double right_pos) {
    if (axis != E_AXIS) {
        return;
    }

    // LOG_I("addFuncParamsExtend: axis%d!\n", axis);

    if (max_size < getSize()) {
        max_size = getSize();
    }
    if (ABS(last_pos_e - right_pos) < EPSILON) {
        type = 0;
    }

    if (type == 0) {
        int func_params_use_tmp = func_params_use;
        if (last_is_zero && func_params_use_tmp != func_params_head && func_params_use_tmp != prevFuncParamsIndex(func_params_head)) {
            FuncParamsExtend &f_p = funcParamsExtend[prevFuncParamsIndex(func_params_head)];

            if (ABS(f_p.right_pos - right_pos)>1) {
                LOG_I("error type: %lf, %lf, a: %d\n", f_p.right_pos, right_pos, axis);
            }

            f_p.right_time = right_time;
            f_p.right_pos = right_pos;

            last_time = right_time;
            last_pos_e = right_pos;

            return;
        } else {
            last_is_zero = true;
        }
    } else {
        last_is_zero = false;
    }


    FuncParamsExtend &f_p = funcParamsExtend[func_params_head];
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
    last_pos_e = right_pos;

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

bool FuncManager::getNextPosTimeEextend(int delta_step, int8_t *dir, float& mm_to_step, float& half_step_mm) {
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

    FuncParamsExtend *func_params = &funcParamsExtend[func_params_use];
    int8_t type = funcParamsTypes[func_params_use];

    int next_step = print_step;
    double next_pos = print_pos_e;
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

        func_params = &funcParamsExtend[func_params_use];
        type = funcParamsTypes[func_params_use];
    }

    if (func_params_tail != func_params_use) {
        func_params_tail = prevFuncParamsIndex(func_params_use);
    }

    if (func_params_use == func_params_head) {
        return false;
    }

    time_double_t next_time = left_time + getTimeByFuncParamsExtend(func_params, type, next_pos, func_params_use);

    print_time = next_time;
    print_pos_e = next_pos;
    print_step = next_step;

    if (average_count == 0 && IS_ZERO(func_params->a)) {
        if (type > 0) {
            int count = FLOOR((func_params->right_pos + EPSILON - next_pos));
            if (count > 0) {
                average_count = count;
                average_step = delta_step;
                average_delta_time = average_step / func_params->b;
                average_print = print_time;
            }
        } else if (type < 0) {
            int count = FLOOR((next_pos - func_params->right_pos + EPSILON));
            if (count > 0) {
                average_count = count;
                average_step = -delta_step;
                average_delta_time = average_step / func_params->b;
                average_print = print_time;
            }
        }
    }

    return true;
}