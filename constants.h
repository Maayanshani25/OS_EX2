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

#define QUANTUM_ERROR_MESSAGE "Error: quantum must be positive.\n"
#define INITIALIZATION_ERROR_MESSAGE "Error: uthread_init called more than once.\n"
#define NO_THREAD_ERROR_MESSAGE "Error: invalid thread ID.\n"
#define ENTRY_POINT_NULL_ERROR_MESSAGE "Error: entry_point cannot be NULL.\n"
#define MAX_NUMBER_EXCEED_ERROR_MESSAGE "Error: maximum number of threads reached.\n"
#define UNINITIALIZED_ERROR_MESSAGE "Error: uthread_init must be called before any other function.\n"
#define THREAD_NOT_FOUND_ERROR "Error: thread not found.\n"
#define BLOCK_MAIM_THREAD_ERROR "Error: cannot block the main thread.\n"
#define NUM_QUANTUMS_ERROR "Error: number of quantums must be positive.\n"
#define MAIN_THREAD_SLEEP_ERROR "Error: main thread cannot call sleep.\n"
#define SIGACTION_ERROR "Error: sigaction failed.\n"
#define SETITIMER_ERROR "Error: setitimer failed.\n"


#endif // CONSTANTS_H
