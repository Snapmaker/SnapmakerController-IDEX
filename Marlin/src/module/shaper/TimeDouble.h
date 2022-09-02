#pragma once

#define EPSILON 0.000001f

class TimeDouble {
  private:
    int i = 0;
    float d = 0;

  public:
    TimeDouble(){};

    TimeDouble(int i) : i(i) {}

    TimeDouble(float d) : d(d) {
        checkCarry();
    }

    inline void checkCarry() {
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
        return *this;
    }

    TimeDouble& operator= (float d) {
        this->d = d;
        checkCarry();
        return *this;
    }

    float operator-(TimeDouble& time_double) const {
        float res = (float)(i - time_double.i);
        res += d - time_double.d;
        return res;
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
};

typedef TimeDouble time_double_t;