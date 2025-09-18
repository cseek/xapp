<!--
 * @Author: aurson jassimxiong@gmail.com
 * @Date: 2025-09-14 17:33:37
 * @LastEditors: aurson jassimxiong@gmail.com
 * @LastEditTime: 2025-09-18 15:39:57
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
-->
```text
 __ _____ ____  ___ 
 \ \ / _ `/ _ \/ _ \
/_\_\\_,_/ .__/ .__/
        /_/  /_/ 
```

***

# Index

[📑 1. Project Introduction](#project-introduction)

[📒 2. Directory Structure](#directory-structure)

[💻 3. Development Environment](#development-environment)

[🔨 4. Compiling the Code](#compiling-the-code)

[💯 5. Unit Testing](#unit-testing)

[🔬 6. Static Check](#static-check)

[🖥️ 7. Development Guide](#development-guide)

***

## Project Introduction
This is a <font color = green>C/C++ app demo</font>.

## Directory Structure

```bash
.
├── cmake         # cmake
├── deps          # Dependencies
├── src           # Source Code
├── tools         # Tools
└── tests
   ├── regression # Regression Tests
   └── unittest   # Unit Tests
```

## Development Environment
```bash
sudo apt update
sudo apt install cmake make xlstproc git
```
+ Build System: Ubuntu 20.04 or higher;
+ Build System: CMake 3.10 or higher;
+ Cross-Compiler: <font color = red>Specified in cmake/toolchain</font>.

## Compiling the Code
```bash
./compile.sh
```

## Unit Testing
This project uses the doctest framework, which is an open-source, lightweight unit testing framework.
```bash
./xapp_unittest --help        # View all usage options
./xapp_unittest -ltc          # List all test cases
./xapp_unittest -lts          # List all test suites
./xapp_unittest -ts=something # Test the 'something' suite
./xapp_unittest -tc=something # Test the 'something' case
./xapp_unittest               # Run all tests
```

## Static Check
This project uses the cppcheck tool.

```bash
./tools/cppcheck.sh
```

## Development Guide
+ [wiki](todo)
+ After cloning the code locally, please run the following command to install the mandatory commit message guidelines.

```bash
./.githook.sh
```

## Completion Status
+ TODO
***
👩‍💻 <font color = red> Issue Reporting: </font> jassimxiong@gmail.com

<p align="lift"> Copyright (c) 2025 by Aurson, All Rights Reserved.<p>