#include <unity.h>
#include "array.hpp"

using util::array;

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void array_elems() {
    const array<int, 4> a({1, 2, 3, 4});

    TEST_ASSERT_EQUAL(a[3], 4);

    int i;
    for (i = 0; i < 3; ++i) {
        TEST_ASSERT_EQUAL(a[i], i + 1);
    }

    i = 1;
    for (const auto& elem : a) {
        TEST_ASSERT_EQUAL(elem, i++);
    }
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(array_elems);

    UNITY_END();
}