#pragma once

#include "defines.hpp"

namespace util {

template <typename T, int N>
class array
{
    static_assert(N > 0, "Cannot have a negative or 0-sized array!");

public:
    using iterator = T*;
    using const_iterator = const T*;

    constexpr T& operator[](int idx) { return const_cast<T&>(const_cast<const array*>(this)->operator[](idx)); }

    constexpr const T& operator[](int idx) const
    {
        DEBUG_ASSERT(idx >= 0 && idx < N, "Array index out of bounds!");
        return data[idx];
    }

    // Iterator helper methods
    constexpr const_iterator cbegin() const { return data; }
    constexpr const_iterator cend() const { return data + N; }
    constexpr iterator begin() { return data; }
    constexpr const_iterator begin() const { return cbegin(); }
    constexpr iterator end() { return data + N; }
    constexpr const_iterator end() const { return cend(); }

    [[nodiscard]] constexpr int size() const { return N; }

    // Publicly accessible to make aggregate initialization possible
    // Should not be used outside of this file
    T data[N]{}; // NOLINT(modernize-avoid-c-arrays)
};

template <typename T0, typename... T>
array(T0, T...) -> array<T0, sizeof...(T) + 1>;

} // namespace util