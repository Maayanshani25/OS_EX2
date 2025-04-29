#include "uthreads.h"
#include "stdio.h"
#include <signal.h>
#include <unistd.h>

void f()
{
  uthread_terminate(0);
}

int main(int argc, char **argv)
{
  uthread_init (999999);
  uthread_spawn (f);
  kill(getpid(), SIGVTALRM);
}