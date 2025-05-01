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
#include <sys/time.h>
#include <memory>

#define AFTER_CONTEXT_SWITCH 1
/**
 * todo:
 * 1. Re-design all initialize error checks
 */

#ifdef __x86_64__
/* code for 64 bit Intel arch */

typedef unsigned long address_t;
#define JB_SP 6
#define JB_PC 7

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

#else
/* code for 32 bit Intel arch */

typedef unsigned int address_t;
#define JB_SP 4
#define JB_PC 5


/* A translation is required when using an address of a variable.
   Use this as a black box in your code. */
address_t translate_address(address_t addr)
{
    address_t ret;
    asm volatile("xor    %%gs:0x18,%0\n"
                 "rol    $0x9,%0\n"
    : "=g" (ret)
    : "0" (addr));
    return ret;
}


#endif

typedef struct Thread {
    int id;
    ThreadState state;
    char* stack_ptr;
    sigjmp_buf env;
    int quantum_count; // Number of quanta the thread has run

    ~Thread() {
        delete[] stack_ptr;
    }

} Thread;

int quantum_usecs_total = 0;
int tid_to_delete = -1;
InitializationState initialized = UNINITIALIZED;
int running_tid = MAIN_THREAD_ID; // CHANGED: use thread id instead of pointer
//std::queue<Thread *> readyQueue;
std::queue<int> readyQueue;
std::priority_queue<int, std::vector<int>, std::greater<int>> available_ids;
std::unordered_map<int, std::unique_ptr<Thread>> thread_map;
std::unordered_map<int, int> sleeping_threads;

static struct itimerval timer;
sigset_t signal_set;

void block_timer_signal();//Tomer
void unblock_timer_signal();
void remove_thread_from_ready_queue(int tid);
std::unique_ptr<Thread> setup_thread(int tid, thread_entry_point entry_point);
void timer_handler(int sig);

void alert_sleep_block();
void reset_timer();
void self_terminate_thread(int tid);
/**
 * Prevents delivery of SIGVTALRM by blocking it.
 */
void block_timer_signal() {
    int masking_success = sigprocmask(SIG_BLOCK, &signal_set, nullptr) == -1;
    if (masking_success) {
        std::cerr << "Failed to block timer signal.\n";
        exit(EXIT_FAILURE);
    }
}

/**
 * Sets up the signal set to only include SIGVTALRM.
 */
void init_signal_mask() {
    if (sigemptyset(&signal_set) == -1) {
        std::cerr << "Error: Could not clear signal set.\n";
        exit(EXIT_FAILURE);
    }

    if (sigaddset(&signal_set, SIGVTALRM) == -1) {
        std::cerr << "Error: Failed to add SIGVTALRM to signal set.\n";
        exit(EXIT_FAILURE);
    }
}

/**
 * Re-enables SIGVTALRM delivery by unblocking it.
 */
void unblock_timer_signal() {

    int unmasking_failed =
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr) == -1;
    if (unmasking_failed) {
        std::cerr << "Failed to unblock timer signal.\n";
        exit(EXIT_FAILURE);
    }
}

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
    init_signal_mask();
    std::unique_ptr<Thread> main_thread(new Thread());
    main_thread->id = MAIN_THREAD_ID;
    main_thread->state = RUNNING;
    main_thread->quantum_count = 1;

    thread_map[MAIN_THREAD_ID] = std::move(main_thread);
    readyQueue.push(MAIN_THREAD_ID);

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
        exit(EXIT_FAILURE);
    }

    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = quantum_usecs;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = quantum_usecs;
    reset_timer();

    return SUCCESS;
}

int uthread_spawn(thread_entry_point entry_point) {
    block_timer_signal();
    if (initialized == UNINITIALIZED) {
        std::cerr << UNINITIALIZED_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }
    if (entry_point == NULL) {
        std::cerr << ENTRY_POINT_NULL_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }
    if (available_ids.empty()) {
        std::cerr << MAX_NUMBER_EXCEED_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    int new_tid = available_ids.top();
    available_ids.pop();

    auto new_thread = setup_thread(new_tid, entry_point);

    readyQueue.push(new_tid);
    thread_map[new_tid] = std::move(new_thread);

    unblock_timer_signal();

    return new_tid;
}

int uthread_terminate(int tid) {
    block_timer_signal();

    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }
    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    if (tid == MAIN_THREAD_ID) {
        unblock_timer_signal();
        exit(EXIT_SUCCESS);
    }

    if (tid == running_tid) {
        self_terminate_thread(tid);
        unblock_timer_signal();
        return 0;
    }

    //if the thread is in ready, remove from ready queue
    if (thread_map[tid]->state == READY) {
        remove_thread_from_ready_queue(tid);
    }
    // remove from all threads
    thread_map.erase(tid);
    unblock_timer_signal();
    return SUCCESS;
}
void self_terminate_thread(int tid) {
    tid_to_delete = running_tid;
    readyQueue.pop();

    //TODO: remove before submition
    if (readyQueue.empty()) {
        thread_map.erase(tid_to_delete);
        std::cerr << "No more threads to schedule after termination.\n";
        exit(EXIT_SUCCESS);
    }

    quantum_usecs_total++;
    running_tid = readyQueue.front();

    Thread *running_thread = thread_map[running_tid].get();
    running_thread->state = RUNNING;
    running_thread->quantum_count++;

    reset_timer();
    unblock_timer_signal();
    siglongjmp(running_thread->env, AFTER_CONTEXT_SWITCH);
}

int uthread_block(int tid) {
    block_timer_signal();
    if (initialized == UNINITIALIZED) {
        std::cerr << UNINITIALIZED_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    if (tid == MAIN_THREAD_ID) {
        std::cerr << BLOCK_MAIM_THREAD_ERROR << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    Thread *thread_to_block = thread_map[tid].get();

    // Already blocked: do nothing
    if (thread_to_block->state == BLOCKED) {
        unblock_timer_signal();
        return SUCCESS;
    }

    // Case: thread_to_block blocks itself
    if (tid == running_tid) {
        thread_to_block->state = BLOCKED;
        readyQueue.pop();
        int new_running_tid = readyQueue.front();
        auto nextThread = thread_map[new_running_tid].get();
        nextThread->state = RUNNING;

        int ret_val = sigsetjmp(thread_map[running_tid]->env, 1);
        if (ret_val == 0) {
            running_tid = nextThread->id;
            quantum_usecs_total++;
            nextThread->quantum_count++;
            reset_timer();
            unblock_timer_signal();
            siglongjmp(nextThread->env, 1);
        }

        if (ret_val == AFTER_CONTEXT_SWITCH) {
            if (tid_to_delete != -1) {
                thread_map.erase(tid_to_delete); // Now safe
                available_ids.push(tid_to_delete);
                tid_to_delete = -1;
            }
            unblock_timer_signal();
        }
        return SUCCESS; // not reached, here for compiler compatibility
    }

    // Case: blocking another thread_to_block that's in READY state
    //if (thread_to_block->state == READY)
    remove_thread_from_ready_queue(tid);
    thread_to_block->state = BLOCKED;
    unblock_timer_signal();
    return SUCCESS;
}

int uthread_resume(int tid) {
    block_timer_signal();
    if (initialized == UNINITIALIZED) {
        std::cerr << UNINITIALIZED_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    Thread *thread = thread_map[tid].get();

    if (thread->state == BLOCKED) {
        thread->state = READY;
        if (sleeping_threads.find(tid) == sleeping_threads.end()) {
            readyQueue.push(thread->id);
        }
    }

    // If it was already READY or RUNNING, do nothing
    unblock_timer_signal();
    return SUCCESS;
}

int uthread_sleep(int num_quantums) {
    block_timer_signal();

    if (num_quantums <= 0) {
        std::cerr << NUM_QUANTUMS_ERROR << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    if (running_tid == MAIN_THREAD_ID) {
        std::cerr << MAIN_THREAD_SLEEP_ERROR << std::endl;
        unblock_timer_signal();
        return ERROR;
    }

    // Register the thread as sleeping
    sleeping_threads[running_tid] = num_quantums;
    thread_map[running_tid]->state = READY;
    readyQueue.pop();

    int new_running_tid = readyQueue.front();
    auto nextThread = thread_map[new_running_tid].get();
    nextThread->state = RUNNING;

    int ret_val = sigsetjmp(thread_map[running_tid]->env, 1);
    if (ret_val == 0) {
        running_tid = nextThread->id;
        quantum_usecs_total++;
        nextThread->quantum_count++;
        reset_timer();
        unblock_timer_signal();
        siglongjmp(nextThread->env, 1);
    }

    if (ret_val == AFTER_CONTEXT_SWITCH) {
        if (tid_to_delete != -1) {
            thread_map.erase(tid_to_delete); // Now safe
            available_ids.push(tid_to_delete);
            tid_to_delete = -1;
        }
        unblock_timer_signal();
    }
    return SUCCESS; // not reached, here for compiler compatibility
}

int uthread_get_tid() {
    return running_tid;
}

int uthread_get_total_quantums() {
    block_timer_signal();
    int total_quantums = quantum_usecs_total;
    unblock_timer_signal();
    return quantum_usecs_total;
}

int uthread_get_quantums(int tid) {
    block_timer_signal();
    if (tid < 0 || tid >= MAX_THREAD_NUM) {
        std::cerr << NO_THREAD_ERROR_MESSAGE << std::endl;
        unblock_timer_signal();
        return ERROR;
    }
    if (thread_map.find(tid) == thread_map.end()) {
        std::cerr << THREAD_NOT_FOUND_ERROR << std::endl;
        unblock_timer_signal();
        return ERROR;
    }
    int quantum_count = thread_map[tid]->quantum_count;
    unblock_timer_signal();
    return quantum_count;

}

/**
 * @brief Timer handler function that is called when the timer expires.
 *
 * This function updates the sleeping threads and switches to the next thread in the ready queue.
 *
 * @param sig The signal number (not used).
 */
void timer_handler(int sig) {
    if (sig != SIGVTALRM) {
        return;
    }

    block_timer_signal();
    alert_sleep_block();

    readyQueue.pop();
    readyQueue.push(running_tid);
    thread_map[running_tid]->state = READY;

    int new_running_tid = readyQueue.front();
    auto nextThread = thread_map[new_running_tid].get();
    nextThread->state = RUNNING;

    int ret_val = sigsetjmp(thread_map[running_tid]->env, 1);
    if (ret_val == 0) {
        running_tid = nextThread->id;
        quantum_usecs_total++;
        nextThread->quantum_count++;
        reset_timer();
        siglongjmp(nextThread->env, 1);
    }

    if (ret_val == AFTER_CONTEXT_SWITCH) {
        if (tid_to_delete != -1) {
            thread_map.erase(tid_to_delete); // Now safe
            available_ids.push(tid_to_delete);
            tid_to_delete = -1;
        }
        unblock_timer_signal();
    }
}

void reset_timer() {
    if (setitimer(ITIMER_VIRTUAL, &timer, nullptr) == -1) {
        std::cerr << "Failed to reset virtual timer.\n";
        exit(EXIT_FAILURE);
    }

}

void alert_sleep_block() {
    // First, update sleeping threads
    std::vector<int> threads_to_wake;
    for (auto &entry: sleeping_threads) {
        entry.second--;
        if (entry.second <= 0 && thread_map[entry.first]->state == READY) {
            threads_to_wake.push_back(entry.first);
        }
    }
    for (int tid: threads_to_wake) {
        sleeping_threads.erase(tid);
        thread_map[tid]->state = READY;
        readyQueue.push(tid);
    }
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
std::unique_ptr<Thread> setup_thread(int new_tid, thread_entry_point entry_point) {
    std::unique_ptr<Thread> new_thread(new Thread());
    new_thread->id = new_tid;
    new_thread->state = READY;
    new_thread->quantum_count = 0;

    new_thread->stack_ptr = new char[STACK_SIZE];
    address_t sp = (address_t)(new_thread->stack_ptr + STACK_SIZE - sizeof(address_t));
    address_t pc = (address_t) entry_point;

    sigsetjmp(new_thread->env, 1);
    (new_thread->env->__jmpbuf)[JB_SP] = translate_address(sp);
    (new_thread->env->__jmpbuf)[JB_PC] = translate_address(pc);
    sigemptyset(&new_thread->env->__saved_mask);

    return new_thread;
}


void remove_thread_from_ready_queue(int tid) {

    std::queue<int> tempQueue;
    while (!readyQueue.empty()) {
        int current_tid = readyQueue.front();
        readyQueue.pop();
        if (current_tid != tid) {
            tempQueue.push(current_tid);
        }
    }
    readyQueue = std::move(tempQueue);
}

