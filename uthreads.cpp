#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include "constants.h"
#include "uthreads.h"
#include <queue>
#include <iostream>
#include <setjmp.h>
#include <unordered_map>
#include <setjmp.h>
#include <sys/time.h>
#include <signal.h>

/**
 * todo:
 * 1. Re-design all initialize error checks
 */

typedef unsigned long address_t;
#define JB_SP 6
#define JB_PC 7

typedef struct Thread {
    int id;
    ThreadState state;
    char stack[STACK_SIZE];
    sigjmp_buf env;
    int quantum_count; // Number of quanta the thread has run
    // Add more fields later:  context, etc.

} Thread;

int quantum_usecs_total = 0;
InitializationState initialized = UNINITIALIZED;
Thread *runningThread;
std::queue<Thread *> readyQueue;
std::priority_queue<int, std::vector<int>, std::greater<int>> available_ids;
std::unordered_map<int, Thread *> thread_map;
std::unordered_map<int, int> sleeping_threads;

static struct itimerval timer;

/* A translation is required when using an address of a variable.
   Use this as a black box in your code. */
address_t translate_address(address_t addr) {
    address_t ret;
    asm volatile("xor    %%fs:0x30,%0\n"
                 "rol    $0x11,%0\n"
        : "=g" (ret)
        : "0" (addr));
    return ret;
}

void remove_thread_from_ready_queue(int tid);
void free_allocated_memory();
void switch_to_next_thread();
Thread* setup_thread(int tid, thread_entry_point entry_point);
void timer_handler(int sig);

int uthread_init(int quantum_usecs) {
    if (initialized) {
        std::cerr << INITIALIZATION_ERROR_MESSAGE << std::endl;
        return ERROR;
    }

    if (quantum_usecs <= 0) {
        std::cerr << QUANTUM_ERROR_MESSAGE << std::endl;
        return ERROR;
    }

    quantum_usecs_total = 1; // After init, already inside first quantum
    initialized = INITIALIZED;

    Thread *main_thread = new Thread{
        .id = MAIN_THREAD_ID,
        .state = RUNNING,
        .quantum_count = 1
    };

    runningThread = main_thread;
    thread_map[MAIN_THREAD_ID] = main_thread;

    for (int i = 1; i < MAX_THREAD_NUM; ++i) {
        available_ids.push(i);
    }

    // Setup the timer here:
    struct sigaction sa = {0};
    sa.sa_handler = &timer_handler; // You will define timer_handler
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGVTALRM, &sa, NULL) < 0) {
        std::cerr << SIGACTION_ERROR << std::endl;
        exit(1);
    }

    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = quantum_usecs;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = quantum_usecs;
    if (setitimer(ITIMER_VIRTUAL, &timer, NULL) < 0) {
        std::cerr << SETITIMER_ERROR << std::endl;
        exit(1);
    }

    return SUCCESS;
}

int uthread_spawn(thread_entry_point entry_point) {
    if (initialized == UNINITIALIZED) {
        std::cerr << UNINITIALIZED_ERROR_MESSAGE << std::endl;
        return ERROR;
    }
    if (entry_point == NULL) {
        std::cerr << ENTRY_POINT_NULL_ERROR_MESSAGE << std::endl;
        return ERROR;
    }
    if (available_ids.empty()) {
        std::cerr << MAX_NUMBER_EXCEED_ERROR_MESSAGE << std::endl;
        return ERROR;
    }

    // Get the smallest available thread ID
    int new_tid = available_ids.top();
    available_ids.pop();

    Thread *new_thread = setup_thread(new_tid, entry_point);

    // Add the thread to the ready queue
    readyQueue.push(new_thread);

    // Add the thread to the thread map
    thread_map[new_tid] = new_thread;

    return new_tid;
}

int uthread_terminate(int tid) {
    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        return ERROR;
    }
    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        return ERROR;
    }

    if (tid == MAIN_THREAD_ID) {
        // Free all threads
        free_allocated_memory();
        exit(0); // terminate the process
    }

    if (tid == runningThread->id) {
        // Self-terminate
        delete runningThread;
        thread_map.erase(tid);

        switch_to_next_thread();
        // Doesn't return
    }

    // If the thread is READY, remove it from the readyQueue
    if (thread_map[tid]->state == READY) {
        remove_thread_from_ready_queue(tid);
    }

    // Free and remove the thread
    delete thread_map[tid];
    thread_map.erase(tid);

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

    Thread *thread = thread_map[tid];

    // Already blocked: do nothing
    if (thread->state == BLOCKED) {
        return SUCCESS;
    }

    // Case: thread blocks itself
    if (tid == runningThread->id) {
        thread->state = BLOCKED;

        switch_to_next_thread();
    }

    // Case: blocking another thread that's in READY state
    if (thread->state == READY) {
        remove_thread_from_ready_queue(tid);
    }

    thread->state = BLOCKED;
    return SUCCESS;
}

int uthread_resume(int tid) {
    if (initialized == UNINITIALIZED) {
        std::cerr << UNINITIALIZED_ERROR_MESSAGE << std::endl;
        return ERROR;
    }

    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        return ERROR;
    }

    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        return ERROR;
    }

    Thread *thread = thread_map[tid];

    if (thread->state == BLOCKED) {
        thread->state = READY;
        readyQueue.push(thread);
    }

    // If it was already READY or RUNNING, do nothing
    return SUCCESS;
}

int uthread_sleep(int num_quantums) {
    if (num_quantums <= 0) {
        std::cerr << NUM_QUANTUMS_ERROR << std::endl;
        return ERROR;
    }

    if (runningThread->id == MAIN_THREAD_ID) {
        std::cerr << MAIN_THREAD_SLEEP_ERROR << std::endl;
        return ERROR;
    }

    int tid = runningThread->id;

    // Mark thread as sleeping
    sleeping_threads[tid] = num_quantums;

    // Block the running thread
    runningThread->state = BLOCKED;

    return SUCCESS;
}

void free_allocated_memory() {
    for (auto &pair: thread_map) {
        delete pair.second;
    }
    thread_map.clear();
}

int uthread_get_tid() {
    return runningThread->id;
}

int uthread_get_total_quantums() {
    return quantum_usecs_total;
}

int uthread_get_quantums(int tid) {
    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        return ERROR;
    }
    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        return ERROR;
    }
    return thread_map[tid]->quantum_count;

}

/**
 * @brief Timer handler function that is called when the timer expires.
 *
 * This function updates the sleeping threads and switches to the next thread in the ready queue.
 *
 * @param sig The signal number (not used).
 */
void timer_handler(int sig) {
    // First, update sleeping threads
    std::vector<int> threads_to_wake;
    for (auto &entry: sleeping_threads) {
        entry.second--;
        if (entry.second <= 0) {
            threads_to_wake.push_back(entry.first);
        }
    }
    for (int tid: threads_to_wake) {
        sleeping_threads.erase(tid);
        Thread *t = thread_map[tid];
        t->state = READY;
        readyQueue.push(t);
    }

    // ⬇️ First: increment running thread because it finished a quantum
    runningThread->quantum_count++;
    quantum_usecs_total++;

    if (!readyQueue.empty()) {
        readyQueue.push(runningThread); // Push current back to ready
        switch_to_next_thread();        // Switch
    }
    // Else: if no other thread, runningThread keeps running (no switch)
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
Thread* setup_thread(int new_tid, thread_entry_point entry_point) {

    Thread* new_thread = new Thread{
        .id = new_tid,
        .state = READY,
        .quantum_count = 0
    };

    address_t sp = (address_t) (new_thread->stack + STACK_SIZE
                                - sizeof(address_t));
    address_t pc = (address_t) entry_point;

    sigsetjmp(new_thread->env, 1);  // Save the base context
    (new_thread->env->__jmpbuf)[JB_SP] = translate_address(sp);
    (new_thread->env->__jmpbuf)[JB_PC] = translate_address(pc);
    sigemptyset(&new_thread->env->__saved_mask);

    return new_thread;
}

/**
 * @brief Switches to the next thread in the ready queue.
 *
 * Pops the next thread from the ready queue, sets its state to RUNNING,
 * and jumps to its execution context using siglongjmp.
 */
void switch_to_next_thread() {
    int ret_val = sigsetjmp(runningThread->env, 1);
    if (ret_val == 1) {
        // Coming back after siglongjmp -> just continue running
        return;
    }

    // Switch to next thread
    runningThread = readyQueue.front();
    readyQueue.pop();
    runningThread->state = RUNNING;
    siglongjmp(runningThread->env, 1);
}

void remove_thread_from_ready_queue(int tid) {
    std::queue<Thread *> tempQueue;
    while (!readyQueue.empty()) {
        Thread *t = readyQueue.front();
        readyQueue.pop();
        if (t->id != tid) {
            tempQueue.push(t);
        }
    }
    readyQueue = tempQueue;
}
