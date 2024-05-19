#include <unity.h>
#include "map.hpp"

using util::map;

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void map_init() {
    const map<int, int, 2> m({{'A', 1}, {'Z', 26}});
    const map<int, int, 2> m2({{'A', 26}, {'Z', 1}});

    TEST_ASSERT_EQUAL(m['A'], 1);
    TEST_ASSERT_EQUAL(m['Z'], 26);

    for (const auto& elem : m) {
        TEST_ASSERT(elem.first && elem.second);
    }

    TEST_ASSERT_EQUAL(m2['A'], 26);
    TEST_ASSERT_EQUAL(m2['Z'], 1);

    for (const auto& elem : m2) {
        TEST_ASSERT(elem.first && elem.second);
    }
}

void map_strings() {
    const map<const char*, int, 2> m({{"hello", 5}, {" world!", 7}});
    map<const char*, int, 2> m2({{"hello", 3}, {" world!", 2}});

    TEST_ASSERT_EQUAL(m["hello"], 5);
    TEST_ASSERT_EQUAL(m[" world!"], 7);

    TEST_ASSERT_EQUAL(m2["hello"], 3);
    TEST_ASSERT_EQUAL(m2[" world!"], 2);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(map_init);
    RUN_TEST(map_strings);

    UNITY_END();
}