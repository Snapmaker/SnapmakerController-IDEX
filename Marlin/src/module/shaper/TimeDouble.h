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

#include "../../MarlinCore.h"

#define EPSILON 0.000001f
#define IS_ZERO(x) (ABS(x) < EPSILON)

class TimeDouble {
  // private:
  public:
    int i = 0;
    float d = 0;

  public:
    TimeDouble(){};

    TimeDouble(int i) : i(i) {
        this->d = 0;
    }

    TimeDouble(float d) : d(d) {
        this->i = 0;
        checkCarry();
    }

    FORCE_INLINE void checkCarry() {
        if (d >= 1) {
            int c = d;
            d = d - c;
            i += c;
        }
        if (this->d <= -1) {
            int c = d;
            d = d - c + 1;
            i += c - 1;
        }
    }

    TimeDouble& operator= (int n) {
        i = n;
        d = 0;
        return *this;
    }

    TimeDouble& operator= (float d) {
        this->i = 0;
        this->d = d;
        checkCarry();
        return *this;
    }

    float operator-(TimeDouble& time_double) const {
        int res_i = (i - time_double.i);
        float res_d = d - time_double.d;
        return res_i + res_d;
    }

    TimeDouble& operator+=(int i) {
        this->i += i;
        return *this;
    }

    TimeDouble& operator+=(TimeDouble& time_double) {
        this->i += time_double.i;
        this->d += time_double.d;
        checkCarry();
        return *this;
    }

    TimeDouble& operator+=(float d) {
        this->d += d;
        checkCarry();
        return *this;
    }

    TimeDouble operator- (float d) const {
        TimeDouble res;
        res.i = this->i;
        res.d = this->d - d;
        res.checkCarry();
        return res;
    }

    TimeDouble operator+(float d) const {
        TimeDouble res;
        res.i = this->i;
        res.d = this->d + d;
        res.checkCarry();
        return res;
    }

    bool operator>(TimeDouble& time_double) const {
        return i != time_double.i ? i > time_double.i : d > time_double.d;
    }

    bool operator>=(TimeDouble& time_double) const {
        return i != time_double.i ? i > time_double.i : d >= time_double.d;
    }

    bool operator<(TimeDouble& time_double) const {
        return i != time_double.i ? i < time_double.i : d < time_double.d;
    }

    bool operator<=(TimeDouble& time_double) const {
        return i != time_double.i ? i < time_double.i : d <= time_double.d;
    }

    bool operator==(TimeDouble& time_double) const {
        return i == time_double.i && d == time_double.d;
    }

    double toDouble() {
        return i + d;
    }

    float toFloat() {
        return i + d;
    }
};

typedef TimeDouble time_double_t;
// typedef double time_double_t;