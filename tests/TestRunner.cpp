#include "TestMain.h"

void run_datastore_tests();
void run_transform_tests();

int main() {
    run_transform_tests();
    run_datastore_tests();

    const int failures = avbtest::failures();
    if (failures == 0) {
        std::printf("All tests passed.\n");
    } else {
        std::printf("%d test(s) failed.\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
