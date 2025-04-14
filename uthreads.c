#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include "constants.h"
#include "uthreads.h"

#define QUANTUM_ERROR_MESSAGE "Error: quantum must be positive.\n"
#define INITIALIZATION_ERROR_MESSAGE "Error: uthread_init called more than once.\n"

typedef struct Thread {
    int id;
    ThreadState state;
    // Add more fields later: stack, context, etc.
} Thread;

static Thread threads[MAX_THREAD_NUM];
static int current_tid = 0;
static int quantum_usecs_global = 0;
static InitializationState initialized = UNINITIALIZED;

int uthread_init(int quantum_usecs) {
    if (initialized) {
        fprintf(stderr, INITIALIZATION_ERROR_MESSAGE);
        return ERROR;
    }

    if (quantum_usecs <= 0) {
        fprintf(stderr, QUANTUM_ERROR_MESSAGE);
        return ERROR;
    }

    quantum_usecs_global = quantum_usecs;
    initialized = INITIALIZED;

    // Initialize main thread (tid == 0)
    threads[0].id = MAIN_THREAD_ID;
    threads[0].state = RUNNING;
    current_tid = 0;

    // todo: check this?
    // No need to create stack for main thread – it uses regular stack.

    return SUCCESS;
}
