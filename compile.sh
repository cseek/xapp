#!/bin/bash
 # @Author: 熊昱卿(Aurson) jassimxiong@gmail.com
 # @Date: 2025-09-14 17:33:37
 # @LastEditors: 熊昱卿(Aurson) jassimxiong@gmail.com
 # @LastEditTime: 2026-04-18 11:58:50
 # @Description: 编译脚本
 # Copyright (c) 2025 by 熊昱卿(Aurson), All Rights Reserved.
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

set -e 

# 默认构建类型， Release 或者 Debug
DEFAULT_BUILD_TYPE="Release"
# 默认构建目录
DEFAULT_BUILD_DIR="build"
# 默认 CMakeLists.txt 的位置
CMAKE_SOURCE_DIR="."

# 支持的PROJECT列表
declare -A SUPPORTED_PROJECTS=(
    ["Aurson"]="Aurson项目"
    ["Google"]="Google项目"
)

# 支持的PLATFORM列表
declare -A SUPPORTED_PLATFORMS=(
    ["Rasp4B"]="树梅派4B平台"
    ["X86"]="X86平台"
)

# 显示帮助信息
show_help() {
    echo "用法:"
    echo "  $0 <PROJECT> <PLATFORM>   编译指定项目和平台"
    echo "  $0 -c | --clean           清理所有构建目录"
    echo "  $0 -h | --help            显示此帮助信息"
    echo ""
    echo "参数:"
    echo "  PROJECT   项目名称"
    echo "  PLATFORM  目标平台"
    echo ""
    echo "支持的PROJECT:"
    for project in "${!SUPPORTED_PROJECTS[@]}"; do
        printf "  %-15s %s\n" "$project" "${SUPPORTED_PROJECTS[$project]}"
    done | sort
    echo ""
    echo "支持的PLATFORM:"
    for platform in "${!SUPPORTED_PLATFORMS[@]}"; do
        printf "  %-15s %s\n" "$platform" "${SUPPORTED_PLATFORMS[$platform]}"
    done | sort
    echo ""
    echo "示例:"
    echo "  $0 Test Rasp4B"
    echo "  $0 --clean (清理所有构建目录)"
    exit 0
}

# 清理所有构建目录
clean_build() {
    echo "========================================"
    if [[ -d "$DEFAULT_BUILD_DIR" ]]; then
        echo "正在删除构建目录: $DEFAULT_BUILD_DIR"
        rm -rf "$DEFAULT_BUILD_DIR"
        echo "清理完成!"
    else
        echo "构建目录不存在: $DEFAULT_BUILD_DIR"
        echo "无需清理"
    fi
    echo "========================================"
    exit 0
}

# 检查参数是否在支持的列表中
check_supported() {
    local value="$1"
    local -n supported_list="$2"
    local list_name="$3"
    
    if [[ -z "${supported_list[$value]}" ]]; then
        echo "错误: 不支持的${list_name} '$value'"
        echo "可用的${list_name}: ${!supported_list[*]}"
        exit 1
    fi
}

# 获取构建目录路径
get_build_dir() {
    local project="$1"
    local platform="$2"
    echo "${DEFAULT_BUILD_DIR}/${project}_${platform}"
}

# 执行CMake配置和构建
execute_build() {
    local project="$1"
    local platform="$2"
    local build_dir="$3"
    
    echo "========================================"
    echo "构建项目: $project (${SUPPORTED_PROJECTS[$project]})"
    echo "构建平台: $platform (${SUPPORTED_PLATFORMS[$platform]})"
    echo "构建目录: $build_dir"
    echo "========================================"
    echo "正在配置并构建项目..."
    cmake -B "$build_dir" \
        -S "$CMAKE_SOURCE_DIR" \
        -DENABLE_CPPCHECK=OFF \
        -DPROJECT="$project" \
        -DPLATFORM="$platform" \
        -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain/Toolchain$platform.cmake" \
        -DCMAKE_BUILD_TYPE="$DEFAULT_BUILD_TYPE" || return 1
    cmake --build "$build_dir" --config "$DEFAULT_BUILD_TYPE" --parallel 8 || return 1     # --parallel $(nproc) 最大线程数构建
    cmake --build "$build_dir" --target package -j8

}

# 处理特殊选项
case "$1" in
    -h|--help)
        show_help
        ;;
    -c|--clean)
        clean_build
        ;;
    *)
        # 继续正常流程
        ;;
esac

# 检查参数数量
if [[ $# -ne 2 ]]; then
    echo "错误: 需要2个参数"
    echo ""
    echo "用法: $0 <PROJECT> <PLATFORM>"
    echo "       $0 -c 清理所有构建"
    echo "       $0 -h 查看帮助"
    exit 1
fi

# 获取参数
PROJECT="$1"
PLATFORM="$2"

# 检查PROJECT是否支持
check_supported "$PROJECT" SUPPORTED_PROJECTS "项目"

# 检查PLATFORM是否支持
check_supported "$PLATFORM" SUPPORTED_PLATFORMS "平台"

# 获取构建目录
BUILD_DIR=$(get_build_dir "$PROJECT" "$PLATFORM")

# 执行构建
execute_build "$PROJECT" "$PLATFORM" "$BUILD_DIR"