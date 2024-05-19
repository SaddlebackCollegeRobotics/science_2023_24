#pragma once

#include <assert.h>

namespace util {

template <typename T, int N>
class array
{
static_assert(N > 0, "Cannot have a negative or 0-sized array!");

public:
    using iterator = T*;
    using const_iterator = const T*;

    constexpr T& operator[](int i)
    {
      return const_cast<T&>(const_cast<const array*>(this)->operator[](i));

    }

    constexpr const T& operator[](int i) const
    {
      assert(i >= 0 && i < N);
      return __data[i];
    }

    // Iterator helper methods
    constexpr const_iterator cbegin() const { return __data; }
    constexpr const_iterator cend() const { return __data + N; }
    constexpr iterator begin() { return __data; }
    constexpr const_iterator begin() const { return cbegin(); }
    constexpr iterator end() { return __data + N; }
    constexpr const_iterator end() const { return cend(); }

    constexpr int size() const { return N; }


  // Publicly accessible to make aggregate initialization possible
  // Should not be used outside of this file
  T __data[N] {};
};

template <typename T0, typename... T>
array(T0, T...) -> array<T0, sizeof...(T) + 1>;


} // namespace util