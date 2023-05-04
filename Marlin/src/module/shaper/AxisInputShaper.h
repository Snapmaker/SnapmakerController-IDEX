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

#include "FuncManager.h"

#define SHAPER_VIBRATION_REDUCTION 20

enum class InputShaperType : int
{
  none = 0,
  ei = 1,
  ei2 = 2,
  ei3 = 3,
  mzv = 4,
  zv = 5,
  zvd = 6,
  zvdd = 7,
  zvddd = 8
};

class ShaperParams
{
public:
  float A[5];
  float T[5];
  int n = 0;
};

class ShaperWindowParams
{
public:
  uint8_t move_index;

  float A, T;
  float a, b, c;

  time_double_t left_time;
  time_double_t time;
};

class ShaperWindow
{
public:
  int n = 0;

  float pos = 0;
  int zero_n = 0;

  time_double_t time = 0;

  ShaperWindowParams params[5];

  FuncParams func_params;

  void updateParamsA() {
    float a = 0;
    for (int i = 0; i < n; i++) {
        a += params[i].a;
    }
    func_params.a = a;
  }

  // void updateParamABC(int i, float start_v, float accelerate, time_double_t start_t, time_double_t left_time, float start_pos, float axis_r);
};

class AxisInputShaper
{
private:
  int axis;

  ShaperParams params;
  ShaperParams shift_params;

  ShaperWindow shaper_window;

  void shiftPulses();

public:
  static AxisInputShaper axis_input_shaper_x;
  static AxisInputShaper axis_input_shaper_y;

  bool is_shaper_window_init = false;

  float frequency = 50;
  float zeta = 0.1;
  InputShaperType type = InputShaperType::zvd;

  float right_delta;
  float left_delta;
  float delta_window;

  float shaped_func_all_time;

  AxisInputShaper(){};

  void setConfig(int type, float frequency, float zeta) {
    this->type = (InputShaperType)type;
    this->frequency = frequency;
    this->zeta = zeta;
  };

  void reset() {
    is_shaper_window_init = false;
  }

  AT_END_OF_TEXT void init();

  void logParams();

  bool isShaped()
  {
    return true;
  }

  void setAxis(int axis)
  {
    this->axis = axis;
  }

  float calcPosition(int move_index, time_double_t time, int move_shaped_start, int move_shaped_end);

  FORCE_INLINE void moveShaperWindowByIndex(FuncManager *func_manager, int move_shaped_start, int move_shaped_end);
  bool moveShaperWindowToNext(FuncManager *func_manager, uint8_t move_shaped_start, uint8_t move_shaped_end);

  bool generateShapedFuncParams(FuncManager *func_manager, uint8_t move_shaper_start, uint8_t move_shaper_end);

  FORCE_INLINE void addFuncParamsToManager(FuncManager *func_manager, float a, time_double_t right_time, float right_pos, float x2, float y1, float y2);
};