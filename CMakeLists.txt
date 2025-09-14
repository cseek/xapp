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

