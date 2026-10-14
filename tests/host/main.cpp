// Runs every TEST() registered in the test_*.cpp files.
#include "check.h"

int main() {
  int failedCases = 0;
  for (const check::Case& test : check::registry()) {
    const int before = check::failures();
    test.fn();
    const bool passed = check::failures() == before;
    if (!passed) ++failedCases;
    printf("%s %s\n", passed ? "ok  " : "FAIL", test.name);
  }
  printf("\n%zu tests, %d failed\n", check::registry().size(), failedCases);
  return failedCases == 0 ? 0 : 1;
}
