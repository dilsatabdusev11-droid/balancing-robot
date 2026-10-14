// Minimal test helpers for the host-side unit tests (no framework needed).
#pragma once

#include <math.h>
#include <stdio.h>

#include <vector>

namespace check {

struct Case {
  const char* name;
  void (*fn)();
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

inline int& failures() {
  static int count = 0;
  return count;
}

inline int registerCase(const char* name, void (*fn)()) {
  registry().push_back({name, fn});
  return 0;
}

}  // namespace check

#define TEST(name)                                                         \
  static void name();                                                      \
  static const int name##_registered = check::registerCase(#name, name);   \
  static void name()

#define CHECK(cond)                                            \
  do {                                                         \
    if (!(cond)) {                                             \
      ++check::failures();                                     \
      printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                                \
  do {                                                                                         \
    const double actual_ = (actual), expected_ = (expected);                                   \
    if (!(fabs(actual_ - expected_) <= (tolerance))) {                                         \
      ++check::failures();                                                                     \
      printf("  FAIL %s:%d: %s = %g, expected %g\n", __FILE__, __LINE__, #actual, actual_,     \
             expected_);                                                                       \
    }                                                                                          \
  } while (0)
