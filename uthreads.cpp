#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include "constants.h"
#include "uthreads.h"
#include <queue>
#include <iostream>



#define QUANTUM_ERROR_MESSAGE "Error: quantum must be positive.\n"
#define INITIALIZATION_ERROR_MESSAGE "Error: uthread_init called more than once.\n"
#define NO_THREAD_ERROR_MESSAGE "Error: invalid thread ID.\n"
#define ENTRY_POINT_NULL_ERROR_MESSAGE "Error: entry_point cannot be NULL.\n"
#define MAX_NUMBER_EXCEED_ERROR_MESSAGE "Error: maximum number of threads reached.\n"

typedef struct Thread {
    int id;
    ThreadState state;
    int quantum_count; // Number of quantums the thread has run
    // Add more fields later: stack, context, etc.


} Thread;

static Thread threads[MAX_THREAD_NUM];
int current_tid = 0;
int next_available_tid = 0;
int quantum_usecs_total = 0;
InitializationState initialized = UNINITIALIZED;
std::queue<Thread> runningQueue;
std::queue<Thread> blockedQueue;
std::queue<Thread> ReadyQueue;
std::priority_queue<int, std::vector<int>, std::greater<int>> available_ids;

static struct itimerval timer;

int uthread_init(int quantum_usecs) {
    if (initialized) {
        fprintf(stderr, INITIALIZATION_ERROR_MESSAGE);
        return ERROR;
    }

    if (quantum_usecs <= 0) {
        fprintf(stderr, QUANTUM_ERROR_MESSAGE);
        return ERROR;
    }

    quantum_usecs_total = quantum_usecs;
    initialized = INITIALIZED;

    // Initialize main thread (tid == 0)
    threads[0].id = MAIN_THREAD_ID;
    threads[0].state = RUNNING;
    threads[0].quantum_count = 1;
    current_tid = 0;

    // Initialize the heap with all thread IDs (1 to MAX_THREAD_NUM - 1)
    for (int i = 1; i < MAX_THREAD_NUM; ++i) {
        available_ids.push(i);
    }


    return SUCCESS;
}

int uthread_spawn(thread_entry_point entry_point){
    if (initialized == UNINITIALIZED){
        fprintf(stderr, INITIALIZATION_ERROR_MESSAGE);
        return ERROR;
    }
    if (entry_point == NULL) {
        fprintf(stderr, ENTRY_POINT_NULL_ERROR_MESSAGE);
        return ERROR;
    }
    if (available_ids.empty()) {
        fprintf(stderr, MAX_NUMBER_EXCEED_ERROR_MESSAGE);
        std::cout<<MAX_NUMBER_EXCEED_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }

    // Get the smallest available thread ID
    int new_tid = available_ids.top();
    available_ids.pop();

    // Initialize the new thread
    threads[new_tid].id = new_tid;
    threads[new_tid].state = READY;
    threads[new_tid].quantum_count = 0;

    // Add the thread to the ready queue
    ReadyQueue.push(threads[new_tid]);

    return new_tid;
}


int uthread_get_tid() {
    return current_tid;
}

int uthread_get_total_quantums() {
    return quantum_usecs_total;
}

int uthread_get_quantums(int tid) {
    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        fprintf(stderr, NO_THREAD_ERROR_MESSAGE);
        return ERROR;
    }
    return threads[tid].quantum_count;
}


//todo: when moving thread to running mode: 1. increase quantum counter
// 2. increase self counter