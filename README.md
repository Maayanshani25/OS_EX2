# OS_EX2

**** Draft answers ****

---

**1. siglongjmp and sigsetjmp:**

**a.**  
`sigsetjmp` saves the current execution state of the program (including registers, stack pointer, and program counter) into a buffer so the program can return to this point later. `siglongjmp` restores the saved state and jumps back to that saved point, as if `sigsetjmp` had just returned.

**b.**  
If `sigsetjmp` is called with a non-zero second argument, it also saves the current signal mask (which signals are blocked). When `siglongjmp` is called, it restores both the saved program state and the saved signal mask. If the second argument to `sigsetjmp` is zero, the signal mask is not saved or restored.

---

**2. User-level threads:**

One common use of user-level threads is in game engines or simulations, where many lightweight tasks (like animations or physics calculations) need to run in parallel. User-level threads are a good choice because they are fast to create, switch between, and manage. They do not require kernel involvement, which makes them efficient for applications that manage many short tasks internally.

---

**3. Chrome’s per-tab process model:**

**Advantages:**
- Each tab runs in its own process, so if one crashes, it doesn’t crash the entire browser.
- Better security and sandboxing, since processes are isolated from each other.
- Each process can run on a different CPU core, allowing real parallelism.

**Disadvantages:**
- Processes use more memory than threads because each has its own memory space.
- More overhead in creating and switching processes compared to threads.
- Communication between processes is more complex and slower than between threads.

---

**4. Interrupts and signals:**

**a.** Use `ps -A | grep Shotwell` to find the PID of the application.  
**b.** In a shell, type `kill pid` (replace pid with the actual number).  
**c.** When you type `kill pid`, your keyboard generates hardware interrupts that the operating system handles to read your input. The shell interprets the `kill` command and makes a system call to request the OS to send a signal (usually SIGTERM) to the process. The OS delivers the signal to the application. If the application does not handle SIGTERM, it is terminated. The application may handle the signal if it has a signal handler defined.

---

**5. Real vs. virtual time:**

**Real time** is the total time from when a program starts running to when it finishes, including waiting for I/O or other delays.  
Example: A program that sleeps for 5 seconds has 5 seconds of real time.

**Virtual time** is the amount of CPU time the process actually used, not counting time spent waiting or sleeping.  
Example: A program that calculates for 2 seconds and then sleeps for 3 seconds has 2 seconds of virtual time.

--- 
