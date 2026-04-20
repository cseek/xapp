/*
 * @Author: 熊昱卿(Aurson) jassimxiong@gmail.com
 * @Date: 2026-04-19 23:35:06
 * @LastEditors: 熊昱卿(Aurson) jassimxiong@gmail.com
 * @LastEditTime: 2026-04-21 02:30:07
 * @Description: 
 * Copyright (c) 2026 by Aurson, All Rights Reserved. 
 */
#pragma once

#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <chrono>

#define COLOR_RED   "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_RESET "\033[0m"

#define CHECK(x)                                                          \
    do {                                                                  \
        auto _t0 = std::chrono::high_resolution_clock::now();             \
        bool _ok = (x);                                                   \
        auto _us = std::chrono::duration_cast<std::chrono::microseconds>( \
                       std::chrono::high_resolution_clock::now() - _t0)   \
                       .count();                                          \
        if(!_ok) {                                                        \
            std::cerr << COLOR_RED << "Check failed:  " << COLOR_RESET    \
                      << #x << "  [" << _us << " us]" << std::endl;       \
            exit(EXIT_FAILURE);                                           \
        } else {                                                          \
            std::cout << COLOR_GREEN << "Check passed: " << COLOR_RESET   \
                      << #x << "  [" << _us << " us]" << std::endl;       \
        }                                                                 \
    } while(0)

static inline bool fEq(double a, double b, double eps = 1e-3) {
    return std::fabs(a - b) < eps;
}

static inline std::vector<uint8_t> toVec(const char *s) {
    return std::vector<uint8_t>(s, s + strlen(s));
}

static inline void vecAppend(std::vector<uint8_t> &dst, const std::vector<uint8_t> &src) {
    dst.insert(dst.end(), src.begin(), src.end());
}

static inline void rawAppend(std::vector<uint8_t> &dst, const uint8_t *data, size_t len) {
    dst.insert(dst.end(), data, data + len);
}
