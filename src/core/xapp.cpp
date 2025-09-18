/*
 * @Author: aurson jassimxiong@gmail.com
 * @Date: 2025-09-14 17:33:37
 * @LastEditors: aurson jassimxiong@gmail.com
 * @LastEditTime: 2025-09-18 15:43:02
 * @Description:
 * Copyright (c) 2025 by Aurson, All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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