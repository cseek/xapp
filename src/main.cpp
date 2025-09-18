/*
 * @Author: aurson jassimxiong@gmail.com
 * @Date: 2025-09-14 17:33:37
 * @LastEditors: aurson jassimxiong@gmail.com
 * @LastEditTime: 2025-09-18 15:43:37
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
#include <iostream>
#include <signal.h>

void print_app_info() {
    std::cout << "   __ _____ ____  ___ "       << std::endl;
    std::cout << "   \\ \\ / _ `/ _ \\/ _ \\ "  << std::endl;
    std::cout << "   /_\\_\\_,_/ .__/ .__/"     << std::endl;
    std::cout << "          /_/  /_/    "       << std::endl;
    std::cout << "────────────────────────────" << std::endl;
    std::cout << "[AppName   ] : " << APP_NAME    << std::endl;
    std::cout << "[BuildDate ] : " << BUILD_DATE  << std::endl;
    std::cout << "[AppVersion] : " << APP_VERSION << std::endl;
    std::cout << "[CommitHash] : " << GIT_HASH    << std::endl;
    std::cout << "─────────────────────────────" << std::endl;
}

int main() {
    print_app_info();
    Aurson::Xapp app;
    app.run();

    return 0;
}