#include "uthreads.h"
#include "stdio.h"
#include <signal.h>
#include <unistd.h>

void f()
{
  uthread_terminate(uthread_get_tid());
}

int main(int argc, char **argv)
{
  uthread_init (999999);
  uthread_spawn (f);
  kill(getpid(),SIGVTALRM);
  uthread_terminate(0);
}