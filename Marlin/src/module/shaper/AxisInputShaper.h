#pragma once

#include "../planner.h"
#include "FuncManager.h"

#define SHAPER_VIBRATION_REDUCTION 20

enum class InputShaperType : int {
    none = 0,
    zv = 1,
    zvd = 2,
    zvdd = 3,
    zvddd = 4,
    mzv = 5,
    ei2 = 6,
    ei3 = 7
};

class ShaperParams {
  public:
    float A[5]{0};
    float T[5]{0};
    int n = 0;
};

class ShaperWindowParams {
  public:
    float A, T;
    float a, b, c;
    time_double_t left_time;
    uint8_t move_index;
    time_double_t time;
};

class ShaperWindow {
  public:
    int n = 0;
    ShaperWindowParams params[5];

    time_double_t time = 0;
    float pos = 0;
    int zero_n = 0;

    FuncParams func_params;

    void updateParamABC(int i, float start_v, float accelerate, time_double_t start_t, time_double_t left_time, float start_pos, float axis_r);
    // void updateABC();
    // void updateABC(float x1, float y1, float x2, float y2);
    // void updateParamLeftTime(time_double_t d);

    void addFuncParams(FuncManager &func_manager, float x2, float y1, float y2);
};

class AxisInputShaper {
  public:
    bool is_shaper_window_init = false;

  private:
    int axis;

    ShaperParams params;
    ShaperParams shift_params;

    ShaperWindow shaper_window;

    void shiftPulses();

  public:
    float frequency = 50;
    float zeta = 0.1;
    InputShaperType type = InputShaperType::zv;

    float right_delta;
    float left_delta;
    float delta_window;

    float shaped_func_all_time;

    AxisInputShaper() {};

    void init();

    bool isShaped() {
        return type != InputShaperType::none;
    }

    void setAxis(int axis) {
        this->axis = axis;
    }


    float calcPosition(int move_index, time_double_t time, int move_shaped_start, int move_shaped_end);

    void moveShaperWindowByIndex(FuncManager &func_manager, int move_shaped_start, int move_shaped_end);
    bool moveShaperWindowToNext(FuncManager &func_manager, uint8_t move_shaped_start, uint8_t move_shaped_end);

    bool generateFuncParams(FuncManager& func_manager, uint8_t move_shaper_start, uint8_t move_shaper_end);
};