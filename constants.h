// constants.h
#ifndef CONSTANTS_H
#define CONSTANTS_H

#define ERROR (-1)
#define SUCCESS 0
#define MAIN_THREAD_ID 0

typedef enum {
    READY,
    RUNNING,
    BLOCKED
} ThreadState;

typedef enum {
    UNINITIALIZED,
    INITIALIZED
} InitializationState;

#define QUANTUM_ERROR_MESSAGE "thread library error: quantum must be positive.\n"
#define INITIALIZATION_ERROR_MESSAGE "thread library error: uthread_init called more than once.\n"
#define NO_THREAD_ERROR_MESSAGE "thread library error: invalid thread ID.\n"
#define ENTRY_POINT_NULL_ERROR_MESSAGE "thread library error: entry_point cannot be NULL.\n"
#define MAX_NUMBER_EXCEED_ERROR_MESSAGE "thread library error: maximum number of threads reached.\n"
#define UNINITIALIZED_ERROR_MESSAGE "thread library error: uthread_init must be called before any other function.\n"
#define THREAD_NOT_FOUND_ERROR "thread library error: thread not found.\n"
#define BLOCK_MAIM_THREAD_ERROR "thread library error: cannot block the main thread.\n"
#define NUM_QUANTUMS_ERROR "thread library error: number of quantums must be positive.\n"
#define MAIN_THREAD_SLEEP_ERROR "thread library error: main thread cannot call sleep.\n"
#define SIGACTION_ERROR "system error: sigaction failed.\n"
#define SIGSETCLEAR_ERROR "system error: Could not clear signal set.\n"
#define SIGADD_ERROR "system error: Failed to add SIGVTALRM to signal set.\n"
#define SETITIMER_ERROR "system error: setitimer failed.\n"
#define SIGNALBLOCK_ERROR "system error: signal blocking failed.\n"
#define SIGNALUNBLOCK_ERROR "system error: signal unblocking failed.\n"



#endif // CONSTANTS_H
