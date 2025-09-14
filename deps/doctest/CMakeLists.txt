project(doctest)
add_library(
    ${PROJECT_NAME} INTERFACE
)
add_library(
    deps::${PROJECT_NAME} ALIAS
    ${PROJECT_NAME}
)
target_include_directories(
    ${PROJECT_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(
    ${PROJECT_NAME} INTERFACE
    $<$<CONFIG:Debug>:-O0 -fvisibility=hidden>
    $<$<CONFIG:Release>:-O2 -DNDEBUG -fvisibility=hidden>
)