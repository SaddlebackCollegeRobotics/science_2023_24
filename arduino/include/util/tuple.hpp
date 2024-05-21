#pragma once

namespace util {

template <int... i>
struct index_sequence
{
};

namespace detail {

template <int i0, int... i>
struct index_sequence_helper : public index_sequence_helper<i0 - 1, i0 - 1, i...>
{
};

template <int... i>
struct index_sequence_helper<0, i...>
{
    using type = index_sequence<i...>;
};

template <int n, typename T>
struct tuple_data
{
    explicit tuple_data(T data)
        : data(data)
    {}

    T data;
};

template <typename Sequence, typename... Types>
struct tuple_helper
{
};

template <int i, int... in, typename T, typename... Rest>
struct tuple_helper<index_sequence<i, in...>, T, Rest...> : tuple_data<i, T>,
                                                            tuple_helper<index_sequence<in...>, Rest...>
{
};

} // namespace detail

template <int N>
using make_index_sequence = typename detail::index_sequence_helper<N>::type;

template <typename... Types>
class tuple : public detail::tuple_helper<make_index_sequence<sizeof...(Types)>, Types...>
{
    [[nodiscard]] constexpr int size() const { return sizeof...(Types); }
};

template <int i, typename T>
constexpr auto get(detail::tuple_data<i, T>& tup)
{
    return tup;
}

template <int i, typename T>
constexpr auto get(const detail::tuple_data<i, T>& tup)
{
    return tup;
}

template <typename T, int i>
constexpr auto get(detail::tuple_data<i, T>& tup)
{
    return tup;
}

template <typename T, int i>
constexpr auto get(const detail::tuple_data<i, T>& tup)
{
    return tup;
}

} // namespace util