#!/bin/bash
###
 # @Author: aurson jassimxiong@gmail.com
 # @Date: 2025-09-14 17:33:37
 # @LastEditors: aurson jassimxiong@gmail.com
 # @LastEditTime: 2025-09-18 15:40:25
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
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${project_dir}/build"
build_type="Release"
toolchain_file="${project_dir}/cmake/toolchain/ToolchainX64.cmake"

cmake -B "${build_dir}/${build_type}" \
      -DENABLE_CPPCHECK=OFF \
      -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}" \
      -DCMAKE_BUILD_TYPE="${build_type}" \
      "${project_dir}" || return 1
cmake --build "${build_dir}/${build_type}" --parallel || return 1
cmake --build "${build_dir}/${build_type}" --target pack --parallel || return 1
