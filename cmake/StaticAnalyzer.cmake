if(ENABLE_CPPCHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        file(GLOB_RECURSE
            CHECK_SRCS
            ${TOP}/src/*.cpp
            ${TOP}/src/*.c
            ${TOP}/src/*.cxx
            ${TOP}/src/*.h
            ${TOP}/src/*.hpp
        )
        execute_process(
            COMMAND cppcheck
            --enable=all
            --inline-suppr
            --language=c++
            --std=c++11
            --quiet
            --force
            --inconclusive
            --suppress=missingIncludeSystem
            --suppress=missingInclude
            ${CHECK_SRCS}
            --xml
            --xml-version=2
            --output-file=${CPPCHECK_REPORT_XML}
        )
        message("Cppcheck finished setting up.")
    else()
        message(SEND_ERROR "Cppcheck requested but executable not found.")
    endif()
endif()
