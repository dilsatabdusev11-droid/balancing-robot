#!/bin/sh
# Builds and runs the host-side unit tests (pure logic, no ESP32 needed).
set -e
cd "$(dirname "$0")"
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Werror -O1 -I../../firmware/self_balancing_robot \
  main.cpp test_*.cpp -o build/host_tests
./build/host_tests
