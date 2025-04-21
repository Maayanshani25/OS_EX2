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
#include <setjmp.h>

/**
 * todo:
 * 1+.User self terminate - Understand it!!!(:
 * 1. Add a working timer
 * 2. Add a function for increasing a thread personal quantum (for get_quantum)
 * 3. Implement sleep function
 * 4. Implement Resume function
 *
 */

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
#define BLOCK_MAIM_THREAD_ERROR "Error: cannot block the main thread.\n"

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
Thread* runningThread;
std::queue<Thread*> blockedQueue;
std::queue<Thread*> ReadyQueue;
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

void remove_thread_from_queue(int tid, std::queue<Thread*>& queue);
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

    runningThread = main_thread;
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
    available_ids.pop();

    //todo: setup_thread(new_tid, entry_point);

    Thread* new_thread = new Thread{
        .id = new_tid,
        .state = READY,
        .quantum_count = 0
        //todo: add env
    };


    // Add the thread to the ready queue
    ReadyQueue.push(new_thread);
    thread_map[new_tid] = new_thread;

    return new_tid;
}



/**
 * @brief Initializes the thread's execution context.
 *
 * Sets up the stack pointer (SP) and program counter (PC) in the thread's saved environment
 * so that when a siglongjmp is performed on this thread, it will start executing from the
 * given entry point with its own stack.
 *
 * @param tid The thread ID to initialize.
 * @param entry_point A function pointer representing the thread's starting function.
 */
void setup_thread(int tid, thread_entry_point entry_point)
{
    Thread* thread = thread_map[tid];  // Get the thread pointer
    address_t sp = (address_t)(thread->stack + STACK_SIZE - sizeof(address_t));
    address_t pc = (address_t) entry_point;

    sigsetjmp(thread->env, 1);  // Save the base context
    (thread->env->__jmpbuf)[JB_SP] = translate_address(sp);
    (thread->env->__jmpbuf)[JB_PC] = translate_address(pc);
    sigemptyset(&thread->env->__saved_mask);
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

    //todo:  If a thread terminates
    // * itself or the main thread is terminated, the function does not return.

    if (thread_map[tid]->state == READY){
        // Remove the thread from the ready queue
        remove_thread_from_queue(tid, ReadyQueue);
    } else if (thread_map[tid]->state == BLOCKED) {
        // Remove the thread from the blocked queue
        remove_thread_from_queue(tid, blockedQueue);
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

int uthread_block(int tid) {
    if (initialized == UNINITIALIZED) {
        std::cerr << UNINITIALIZED_ERROR_MESSAGE << std::endl;
        return ERROR;
    }

    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        return ERROR;
    }

    if (tid == MAIN_THREAD_ID) {
        std::cerr << BLOCK_MAIM_THREAD_ERROR << std::endl;
        return ERROR;
    }

    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        return ERROR;
    }

    Thread* thread = thread_map[tid];

    // Already blocked: do nothing
    if (thread->state == BLOCKED) {
        return SUCCESS;
    }

    // Case: thread blocks itself
    if (tid == runningThread->id) {
        thread->state = BLOCKED;
        blockedQueue.push(thread);

        Thread* next = ReadyQueue.front();
        ReadyQueue.pop();
        next->state = RUNNING;
        runningThread = next;
        return SUCCESS;
    }

    // Case: blocking another thread that's in READY state
    if (thread->state == READY)
    {
        remove_thread_from_queue (tid, ReadyQueue);
    }

    thread->state = BLOCKED;
    blockedQueue.push(thread);
    return SUCCESS;
}


int uthread_get_tid() {
    return runningThread->id;
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


void remove_thread_from_queue(int tid, std::queue<Thread*>& queue) {
    std::queue<Thread*> tempQueue;
    while (!queue.empty()) {
        Thread* t = queue.front();
        queue.pop();
        if (t->id != tid) {
            tempQueue.push(t);
        }
    }
    queue = tempQueue;
}


//todo: when moving thread to running mode: 1. increase quantum counter
// 2. increase self counter