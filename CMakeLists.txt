###
 # @Author: aurson jassimxiong@gmail.com
 # @Date: 2025-09-14 17:33:37
 # @LastEditors: aurson jassimxiong@gmail.com
 # @LastEditTime: 2025-09-18 15:40:42
 # @Description:
 # Copyright (c) 2025 by Aurson, All Rights Reserved.
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
###
cmake_minimum_required(VERSION 3.10)
string(TIMESTAMP BUILD_DATE "%Y%m%d")
set(TOP ${CMAKE_CURRENT_SOURCE_DIR})
########## Project Configuration ##########
set(APP_NAME xapp)
set(APP_VERSION 1.0.0)
set(PKG_NAME "${APP_NAME}-${APP_VERSION}-${BUILD_DATE}-${CMAKE_BUILD_TYPE}")
###########################################
project(${APP_NAME} LANGUAGES C CXX VERSION "${APP_VERSION}")
option(ENABLE_CPPCHECK "Enable static analysis with Cppcheck." OFF)
include(${TOP}/cmake/StaticAnalyzer.cmake)
include(${TOP}/cmake/GitHash.cmake)
include(${TOP}/cmake/Pack.cmake)
add_subdirectory(deps/json)
add_subdirectory(deps/spdlog)
add_subdirectory(deps/doctest)
add_subdirectory(tests/unittest)
add_subdirectory(tests/regression)
add_subdirectory(src)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${PKG_NAME}/bin
)
install(
    TARGETS ${PROJECT_NAME}_unittest
    RUNTIME DESTINATION ${PKG_NAME}/test
)
install(
    TARGETS ${PROJECT_NAME}_regression
    RUNTIME DESTINATION ${PKG_NAME}/test
)

