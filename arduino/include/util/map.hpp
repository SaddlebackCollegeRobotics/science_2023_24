#pragma once

#include "functional.hpp"
#include "pair.hpp"

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

    constexpr V* operator[](const K& key) { return const_cast<V*>(const_cast<const map*>(this)->operator[](key)); }

    constexpr const V* operator[](const K& key) const
    {
        for (auto iter = begin(); iter != end(); ++iter) {
            if (Eq{}(iter->first, key)) {
                return &(iter->second);
            }
        }

        return nullptr;
    }

    // Iterator helper methods
    constexpr const_iterator cbegin() const { return data; }
    constexpr const_iterator cend() const { return data + N; }
    constexpr iterator begin() { return data; }
    constexpr const_iterator begin() const { return cbegin(); }
    constexpr iterator end() { return data + N; }
    constexpr const_iterator end() const { return cend(); }

    [[nodiscard]] constexpr int size() const { return N; }

    underlying_t data[N]{}; // NOLINT(modernize-avoid-c-arrays)
};

} // namespace util