#include <cstdio>

#include "core/Application.h"

int main(int /*argc*/, char** /*argv*/) {
    avb::Application app;
    if (!app.initialize()) {
        std::fprintf(stderr, "Failed to initialize AugmentedVisionBonus2.\n");
        return 1;
    }
    return app.run();
}
