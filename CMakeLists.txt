cmake_minimum_required(VERSION 3.22)
project(OS_EX2)

set(CMAKE_CXX_STANDARD 14)

include_directories(Resources)

add_executable(OS_EX2
        Resources/demo_itimer.c
        Resources/demo_jmp.c
        Resources/demo_singInt_handler.c
        Resources/uthreads.h
        Tests/test0_sanity.cpp
        Tests/test2_two_thread.cpp
        uthreads.cpp
        constants.h
        uthreads.h
        uthreads.cpp)
