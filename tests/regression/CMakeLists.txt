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