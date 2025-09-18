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
if(ENABLE_CPPCHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        file(GLOB_RECURSE
            CHECK_SRCS
            ${TOP}/src/*.cpp
            ${TOP}/src/*.c
            ${TOP}/src/*.cxx
            ${TOP}/src/*.h
            ${TOP}/src/*.hpp
        )
        execute_process(
            COMMAND cppcheck
            --enable=all
            --inline-suppr
            --language=c++
            --std=c++11
            --quiet
            --force
            --inconclusive
            --suppress=missingIncludeSystem
            --suppress=missingInclude
            ${CHECK_SRCS}
            --xml
            --xml-version=2
            --output-file=${CPPCHECK_REPORT_XML}
        )
        message("Cppcheck finished setting up.")
    else()
        message(SEND_ERROR "Cppcheck requested but executable not found.")
    endif()
endif()
