#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include "constants.h"
#include "uthreads.h"
#include <queue>
#include <iostream>
#include <setjmp.h>
//priority queue
#include <unordered_map>




typedef unsigned long address_t;
#define JB_SP 6
#define JB_PC 7

#define QUANTUM_ERROR_MESSAGE "Error: quantum must be positive.\n"
#define INITIALIZATION_ERROR_MESSAGE "Error: uthread_init called more than once.\n"
#define NO_THREAD_ERROR_MESSAGE "Error: invalid thread ID.\n"
#define ENTRY_POINT_NULL_ERROR_MESSAGE "Error: entry_point cannot be NULL.\n"
#define MAX_NUMBER_EXCEED_ERROR_MESSAGE "Error: maximum number of threads reached.\n"
#define UNINITIALIZED_ERROR_MESSAGE "Error: uthread_init must be called before any other function.\n"
#define THREAD_NOT_FOUND_ERROR "Error: thread not found.\n"

typedef struct Thread {
    int id;
    ThreadState state;
    char stack[STACK_SIZE];
    sigjmp_buf env;
    int quantum_count; // Number of quantums the thread has run
    // Add more fields later: stack, context, etc.

} Thread;

int quantum_usecs_total = 0;
InitializationState initialized = UNINITIALIZED;
Thread runningThread;
std::queue<Thread> blockedQueue;
std::queue<Thread> ReadyQueue;
std::priority_queue<int, std::vector<int>, std::greater<int>> available_ids;
std::unordered_map<int, Thread*> thread_map;


static struct itimerval timer;

/* A translation is required when using an address of a variable.
   Use this as a black box in your code. */
address_t translate_address(address_t addr)
{
    address_t ret;
    asm volatile("xor    %%fs:0x30,%0\n"
                 "rol    $0x11,%0\n"
        : "=g" (ret)
        : "0" (addr));
    return ret;
}


int uthread_init(int quantum_usecs) {
    if (initialized) {
        std::cerr<<INITIALIZATION_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }

    if (quantum_usecs <= 0) {
        std::cerr<<QUANTUM_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }

    quantum_usecs_total = quantum_usecs;
    initialized = INITIALIZED;

    Thread* main_thread = new Thread{
        .id = MAIN_THREAD_ID,
        .state = RUNNING,
        .quantum_count = 1
        //todo: add env
    };

    // Add the thread to the ready queue
    ReadyQueue.push(*main_thread);
    thread_map[MAIN_THREAD_ID] = main_thread;


    // Initialize the heap with all thread IDs (1 to MAX_THREAD_NUM - 1)
    for (int i = 1; i < MAX_THREAD_NUM; ++i) {
        available_ids.push(i);
    }


    return SUCCESS;
}

int uthread_spawn(thread_entry_point entry_point){
    if (initialized == UNINITIALIZED){
        std::cerr<<UNINITIALIZED_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }
    if (entry_point == NULL) {
        std::cerr<<ENTRY_POINT_NULL_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }
    if (available_ids.empty()) {
        std::cerr<<MAX_NUMBER_EXCEED_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }

    // Get the smallest available thread ID
    int new_tid = available_ids.top();
    

    Thread* new_thread = new Thread{
        .id = new_tid,
        .state = READY,
        .quantum_count = 0
        //todo: add env
    };

    // Add the thread to the ready queue
    ReadyQueue.push(*new_thread);
    thread_map[new_tid] = new_thread;

    return new_tid;
}

void setup_thread(int tid, char *stack, thread_entry_point entry_point)
{
    // initializes env[tid] to use the right stack, and to run from the function 'entry_point', when we'll use
    // siglongjmp to jump into the thread.
    address_t sp = (address_t) stack + STACK_SIZE - sizeof(address_t);
    address_t pc = (address_t) entry_point;
    sigsetjmp(env[tid], 1);
    (env[tid]->__jmpbuf)[JB_SP] = translate_address(sp);
    (env[tid]->__jmpbuf)[JB_PC] = translate_address(pc);
    sigemptyset(&env[tid]->__saved_mask);
}

int uthread_terminate(int tid){
    if (tid < 0 || tid >= MAX_THREAD_NUM ) {
        std::cerr<<NO_THREAD_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }
    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr<<THREAD_NOT_FOUND_ERROR<<std::endl;
        return ERROR;
    }

    // Free the thread's resources
    delete thread_map[tid];
    thread_map.erase(tid);

    if (tid == MAIN_THREAD_ID) {
        // Terminate the main thread
        exit(0);
    }

    return SUCCESS;
}

int uthread_get_tid() {
    return runningThread.id;
}

int uthread_get_total_quantums() {
    return quantum_usecs_total;
}

int uthread_get_quantums(int tid) {
    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr<<NO_THREAD_ERROR_MESSAGE<<std::endl;
        return ERROR;
    }
    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr<<THREAD_NOT_FOUND_ERROR<<std::endl;
        return ERROR;
    }
    return thread_map[tid]->quantum_count;

}



//todo: when moving thread to running mode: 1. increase quantum counter
// 2. increase self counter