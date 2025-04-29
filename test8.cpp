#include <iostream>

int main() {
  std::string command = "g++ -std=c++11 -fsanitize=address "
                        "-fno-omit-frame-pointer -g uthreads.cpp DoNotRunThis2"
                        ".cpp -o "
                        "test_asan";
  if (system(command.c_str()) != 0)
  {
    printf ("Failed to run test (it doesn't mean you failed the test)\n");
    return 1;
  }
  command = "ASAN_OPTIONS=detect_stack_use_after_return=true > results 2>&1 ./test_asan";
  int result = system(command.c_str());
  if (result != 0)
  {
    printf ("Test failed.\nTry increasing STACK_SIZE to 100000. if it still "
            "fails, you most likely accessed an invalid memory address or "
            "there was a memory leak. hint: check ASAN_OPTIONS\n");
    int n;
    printf ("To see full results, enter 1:\n");
    std::cin >> n;
    if (n == 1)
    {
      command = "cat results";
      system (command.c_str ());
    }
  }
  else
  {
    printf ("Test passed\n");
  }
  command = "rm -f results test_asan";
  system (command.c_str ());
  return result;
}
