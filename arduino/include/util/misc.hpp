#pragma once

#include "array.hpp"
#include "strings.hpp"
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
constexpr util::array<String, N> split(util::string_view str, char delim = ',')
{
    util::array<String, N> result{};
    int resultI = 0;

    auto lastIter = str.begin();
    auto findIter = str.begin();

    while ((findIter = str.find(delim, findIter)) != string_view::npos) {
        result[resultI++] = string_view{lastIter, findIter}.printable();
        findIter++;

        lastIter = findIter;

        if (resultI >= N) {
            return result;
        }
    }

    result[resultI] = string_view{lastIter}.printable();

    return result;
}

} // namespace util