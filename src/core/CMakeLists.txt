file(
    GLOB_RECURSE SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
)
add_library(
    ${PROJECT_NAME}_static STATIC
    ${SRCS}
)
target_include_directories(
    ${PROJECT_NAME}_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
set_target_properties(
    ${PROJECT_NAME}_static PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}
)
target_compile_options(
    ${PROJECT_NAME}_static PUBLIC
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O2 -DNDEBUG>
)
target_link_libraries(
    ${PROJECT_NAME}_static PUBLIC
    deps::spdlog
    pthread
)
target_compile_definitions(
    ${PROJECT_NAME}_static PUBLIC
    GIT_HASH="${GIT_HASH}"
    BUILD_DATE="${BUILD_DATE}"
    APP_VERSION="${APP_VERSION}"
    APP_NAME="${APP_NAME}"
)
