cmake_minimum_required(VERSION 3.10)
project(
    "xdemo"
    LANGUAGES C CXX
    VERSION "0.1.0"
)
set(PACKAGE_INSTALL_DIR "/opt")
set(PACKAGE_VENDOR "Aurson" )
set(PACKAGE_ID "com.aurson.app.${PROJECT_NAME}")
set(PACKAGE_SUMMARY "${PROJECT_NAME} -  demo application")
set(PACKAGE_MAINTAINER "jassimxiong@gmail.com") 
set(TOP ${CMAKE_CURRENT_SOURCE_DIR})
string(TIMESTAMP BUILD_DATE "%Y%m%d")
option(ENABLE_CPPCHECK "Enable static analysis with Cppcheck." OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
endif()
###########################################
include(${TOP}/cmake/StaticAnalyzer.cmake)
include(${TOP}/cmake/GitHash.cmake)
add_subdirectory(deps/json)
add_subdirectory(deps/spdlog)
add_subdirectory(deps/doctest)
# add_subdirectory(tests/unittest)
# add_subdirectory(tests/regression)
add_subdirectory(src)
include(${TOP}/cmake/Pack.cmake)
install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${PACKAGE_INSTALL_DIR}/${PACKAGE_ID}/bin
)