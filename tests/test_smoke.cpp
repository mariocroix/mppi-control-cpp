#include <cassert>
#include <iostream>

int main() {
    std::cout << "Running smoke test...\n";
    assert(2 + 2 == 4);
    std::cout << "Smoke test passed.\n";
    return 0;
}