#include "uthreads.h"
#include "stdio.h"
#include <signal.h>
#include <unistd.h>


void f()
{
  while(true);
}

int main(int argc, char **argv)
{
  printf("If this test doesn't stop running, you've failed the test\n");
  uthread_init (999999);
  uthread_spawn (f);
  kill(getpid(),SIGVTALRM);
  printf("Test passed\n");
  uthread_terminate(0);
}