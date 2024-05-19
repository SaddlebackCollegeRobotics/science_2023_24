#pragma once

#include <assert.h>
#include "intitializer_list.hpp"
#include "pair.hpp"
#include "functional.hpp"

namespace util {

template <typename K, typename V, int N, typename Eq = equals<K>>
class map
{
public:
    using underlying_t = pair<K, V>;
    using iterator = underlying_t*;
    using const_iterator = const underlying_t*;

    // constexpr map() = default;
    // constexpr map(std::initializer_list<underlying_t> init)
    // {
    //     // static_assert(InitSize <= N, "Map cannot be initialized with more elements than N!");

    //     for (int i = 0; i < init.size() && i < N; ++i) {
    //         __data[i] = *(init.begin() + i);
    //     }
    // }

    constexpr V& operator[](const K& key)
    {
        return const_cast<V&>(const_cast<const map*>(this)->operator[](key));
    }

    constexpr const V& operator[](const K& key) const
    {
        for (auto& item : __data) {
            if (Eq{}(item.first, key)) {
                return item.second;
            }
        }
        __builtin_unreachable();
    }

    // Iterator helper methods
    constexpr const_iterator cbegin() const { return __data; }
    constexpr const_iterator cend() const { return __data + N; }
    constexpr iterator begin() { return __data; }
    constexpr const_iterator begin() const { return cbegin(); }
    constexpr iterator end() { return __data + N; }
    constexpr const_iterator end() const { return cend(); }

    constexpr int size() const { return N; }

    underlying_t __data[N] {};


};

} // namespace util