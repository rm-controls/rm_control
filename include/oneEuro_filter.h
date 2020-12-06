//
// Created by liucong on 2020/12/5.
//

#ifndef SRC_RM_INCLUDE_ONEEURO_FILTER_H
#define SRC_RM_INCLUDE_ONEEURO_FILTER_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iterator>
#include <cstdlib>

template <typename T = double>
class low_pass_filter {
public:
    low_pass_filter() : hatxprev(0), xprev(0), hadprev(false) {}
    T operator()(T x, T alpha) {
        T hatx;
        if(hadprev) {
            hatx = alpha * x + (1-alpha) * hatxprev;
        } else {
            hatx = x;
            hadprev = true;
        }
        hatxprev = hatx;
        xprev = x;
        return hatx;
    }

    T hatxprev;
    T xprev;
    bool hadprev;
};

template <typename T = double, typename timestamp_t = double>
class one_euro_filter {
public:
    one_euro_filter(double _freq, T _mincutoff, T _beta, T _dcutoff) : freq(_freq), mincutoff(_mincutoff), beta(_beta), dcutoff(_dcutoff), last_time_(-1) {}
    T operator()(T x, timestamp_t t = -1) {
        T dx = 0;

        if(last_time_ != -1 && t != -1 && t != last_time_) {
            freq = 1.0 / (t - last_time_);
        }
        last_time_ = t;

        if(xfilt_.hadprev)
            dx = (x - xfilt_.xprev) * freq;

        T edx = dxfilt_(dx, alpha(dcutoff));
        T cutoff = mincutoff + beta * std::abs(static_cast<double>(edx));
        return xfilt_(x, alpha(cutoff));
    }
    double freq;
    T mincutoff, beta, dcutoff;
private:
    T alpha(T cutoff) {
        T tau = 1.0 / (2 * M_PI * cutoff);
        T te = 1.0 / freq;
        return 1.0 / (1.0 + tau / te);
    }

    timestamp_t last_time_;
    low_pass_filter<T> xfilt_, dxfilt_;
};

// An example

class randomizer {
public:
    randomizer(int a, int b) : start_(std::min(a, b)), range_(std::max(a,b) - start_) {}
    int operator()() {
        return start_ + rand() % range_;
    }
private:
    int start_, range_;
};

template <typename T>
T rand01() {
    return rand() / static_cast<T>(RAND_MAX);
}

#endif //SRC_ONEEURO_FILTER_H
