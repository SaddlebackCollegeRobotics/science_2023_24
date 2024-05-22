#pragma once

#include "../macros.hpp"

namespace util {

template <typename T, int N>
class array
{
    static_assert(N > 0, "Cannot have a negative or 0-sized array!");

public:
    using iterator = T*;
    using const_iterator = const T*;

    // Initializer to fill an array with a specified value
    static constexpr array fill(const T& value)
    {
        array result;
        for (auto& elem : result) {
            elem = value;
        }
        return result;
    }

    constexpr T& operator[](int idx) { return const_cast<T&>(const_cast<const array*>(this)->operator[](idx)); }

    constexpr const T& operator[](int idx) const
    {
        DEBUG_ASSERT(idx >= 0 && idx < N, "Array index out of bounds! %d (Size = %d)", idx, N);
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

constexpr bool in_range(int idx, int min, int max)
{
    return idx >= min && idx <= max;
}

template <typename T, int N>
constexpr bool in_range(int idx, [[maybe_unused]] const array<T, N>& arr)
{
    return in_range(idx, 0, N - 1);
}

} // namespace util