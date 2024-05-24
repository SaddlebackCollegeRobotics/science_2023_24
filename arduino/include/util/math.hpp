#pragma once

namespace util {

template <typename T, typename U>
constexpr auto max(const T& first, const T& second) -> decltype(first + second)&
{
    if (second > first) {
        return second;
    }

    return first;
}

template <typename T>
constexpr auto clamp(const T& value, const T& max, const T& min)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

} // namespace util