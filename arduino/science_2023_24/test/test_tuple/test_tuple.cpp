#include <unity.h>
#include <tuple.hpp>

using util::tuple;

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void tuple_get() {
    tuple<int, double, char> t = {42, 3.14, '3'};
    auto r = util::get<1>(t);
    auto r2 = util::get<int>(t);
    auto r3 = util::get<char>(t);

    TEST_ASSERT_EQUAL(r, 3.14);
    TEST_ASSERT_EQUAL(r2, 42);
    TEST_ASSERT_EQUAL(r3, '3');
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(tuple_get);

    UNITY_END();
}