#pragma once

// Tiny dependency-free test harness so the backend can be tested in CI without
// pulling in a framework. Each test file contributes CHECK() calls; main()
// reports the number of failures as the process exit code.

#include <cstdio>

namespace avbtest {
inline int& failures() {
    static int n = 0;
    return n;
}
} // namespace avbtest

// Variadic so template commas (e.g. block<3, 1>) inside the condition work.
#define CHECK(...)                                                          \
    do {                                                                    \
        if (!(__VA_ARGS__)) {                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
            ++avbtest::failures();                                          \
        }                                                                   \
    } while (0)
