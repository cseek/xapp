/*
 * @Author: aurson jassimxiong@gmail.com
 * @Date: 2025-09-12 23:03:16
 * @LastEditors: aurson jassimxiong@gmail.com
 * @LastEditTime: 2025-09-14 00:42:50
 * @Description:
 * Copyright (c) 2025 by Aurson, All Rights Reserved.
 */
#include "xapp.h"
#include <future>
#include <iostream>
#include <thread>

namespace Aurson {
    Xapp::Xapp() {
        std::cout << "Creating Xapp object" << std::endl;
    }
    Xapp::~Xapp() {
        std::cout << "Destroying Xapp object" << std::endl;
    }
    void Xapp::run() {
        std::cout << "Xapp is running!" << std::endl;
        std::async(
            std::launch::async,
            []() {
                while (true) {
                    auto now = std::chrono::system_clock::now();
                    auto now_c = std::chrono::system_clock::to_time_t(now);
                    std::cout << "Current time: " << std::ctime(&now_c);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            })
            .wait();
    }
} // namespace Aurson