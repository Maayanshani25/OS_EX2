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



#endif // CONSTANTS_H
