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
project(spdlog)
add_library(
    ${PROJECT_NAME} INTERFACE
)
add_library(
    deps::${PROJECT_NAME} ALIAS
    ${PROJECT_NAME}
)
target_include_directories(
    ${PROJECT_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(
    ${PROJECT_NAME} INTERFACE
    $<$<CONFIG:Debug>:-O0 -fvisibility=hidden>
    $<$<CONFIG:Release>:-O2 -DNDEBUG -fvisibility=hidden>
)