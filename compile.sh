#!/bin/bash
###
 # @Author: aurson xiongyi@asensing.com
 # @Date: 2025-06-10 10:01:34
 # @LastEditors: aurson jassimxiong@gmail.com
 # @Description: 
 # Copyright (c) 2025 by Asensing, All Rights Reserved
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
