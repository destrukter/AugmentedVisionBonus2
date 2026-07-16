#include "TestMain.h"

void run_modelloader_tests();

int main() {
    run_modelloader_tests();

    const int failures = avbtest::failures();
    if (failures == 0) {
        std::printf("All tests passed.\n");
    } else {
        std::printf("%d test(s) failed.\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
