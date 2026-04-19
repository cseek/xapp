file(
    GLOB_RECURSE SRCS
     ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)
add_executable(
    ${PROJECT_NAME}
    ${SRCS}
)
target_link_libraries(
    ${PROJECT_NAME} PRIVATE 
    pthread
    deps::spdlog
)
target_include_directories(
    ${PROJECT_NAME} PRIVATE
    ${TOP}/src
)
target_compile_options(
    ${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
)
target_compile_definitions(
    ${PROJECT_NAME} PRIVATE
    APP_VERSION="${CMAKE_PROJECT_VERSION}"
    ARCH="${CMAKE_SYSTEM_PROCESSOR}"
    GIT_HASH="${GIT_HASH}"
    BUILD_DATE="${BUILD_DATE}"
)