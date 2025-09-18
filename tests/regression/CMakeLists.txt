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
file(
    GLOB_RECURSE TEST_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)
add_executable(
    ${PROJECT_NAME}_regression
    ${TEST_SRCS}
)
set_target_properties(
    ${PROJECT_NAME}_regression PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
target_link_libraries(
    ${PROJECT_NAME}_regression PRIVATE
    deps::doctest
    ${PROJECT_NAME}_static
)