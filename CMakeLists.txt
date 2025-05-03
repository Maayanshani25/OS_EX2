cmake_minimum_required(VERSION 3.22)
project(OS_EX2)

set(CMAKE_CXX_STANDARD 11)

include_directories(Resources)

# Each test should be a separate executable!
add_executable(test0_sanity
        uthreads.cpp
        test0_sanity.cpp
        constants.h
        uthreads.h
)

add_executable(test2_two_thread
        uthreads.cpp
        test2_two_thread.cpp
        constants.h
        uthreads.h
)

add_executable(test1
        uthreads.cpp
        test1.cpp
        constants.h
        uthreads.h
)

add_executable(test2
        uthreads.cpp
        test2.cpp
        constants.h
        uthreads.h
)


add_executable(test3
        uthreads.cpp
        test3.cpp
        constants.h
        uthreads.h
)

add_executable(test4
        uthreads.cpp
        test4.cpp
        constants.h
        uthreads.h
)

add_executable(test5

        test5.cpp

)

add_executable(test6
        uthreads.cpp
        test6.cpp
        constants.h
        uthreads.h
)

add_executable(test7
        uthreads.cpp
        test7.cpp
        constants.h
        uthreads.h
)

add_executable(test8
    
        test8.cpp

)
