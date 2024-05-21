#pragma once

#include "array.hpp"
#include <WString.h>
#include <string.h>

namespace util {

// int find(const char* str, char c)
// {
//     auto iter = str;
//     while (iter != nullptr && *iter != c) {
//         if (*iter == '\0') {
//             return -1;
//         }
//     }

//     return iter - str;
// }

template <int N>
constexpr util::array<String, N> split(const char* str, char delim = ',')
{
    util::array<String, N> result{};
    int resultI = 0;

    const char* lastIter = str;
    const char* findIter = str;
    while ((findIter = strchr(findIter, delim)) != nullptr) {
        result[resultI++] = String(lastIter).substring(0, findIter - lastIter);
        findIter++;

        lastIter = findIter;

        if (resultI >= N) {
            return result;
        }
    }

    result[resultI] = String(lastIter);

    return result;
}

} // namespace util