#!/bin/bash

set -e

echo "building tests..."
cmake --build cmake-build-debug

echo ""
echo "running all tests: (shpritz)"
echo "-------------------"

tests=(
  test0_sanity
  test1
  test2
  test2_two_thread
  test3
  test4
  test5
  test6
  test7
  test8
)

cd cmake-build-debug || exit 1

for test in "${tests[@]}"; do
  if [[ -f "$test" ]]; then
    echo "Running $test:"
    ./"$test"
    echo "-------------------"
  else
    echo " $test not found!"
  fi
done
