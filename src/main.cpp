/*
 * @Author: aurson jassimxiong@gmail.com
 * @Date: 2025-09-12 19:19:25
 * @LastEditors: aurson jassimxiong@gmail.com
 * @LastEditTime: 2025-09-14 00:41:37
 * @Description:
 * Copyright (c) 2025 by Aurson, All Rights Reserved.
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