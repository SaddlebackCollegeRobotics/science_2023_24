#pragma once

#include <functional.hpp>
#include <array.hpp>
#include <tuple.hpp>
#include <Arduino.h>


namespace detail {

constexpr const char** next_str(const char* args[])
{
    while (args != nullptr && **args != '\0') ++args;

    return args;
}

//********************** string to argument impls

template <typename Arg>
/*constexpr*/ inline Arg str_to_arg(const char* str)
{
    return {};
}

template <>
/*constexpr*/ inline int str_to_arg<int>(const char* str)
{
    return atoi(str);
}

template <>
/*constexpr*/ inline float str_to_arg<float>(const char* str)
{
    return atof(str);
}

template <>
/*constexpr*/ inline double str_to_arg<double>(const char* str)
{
    return str_to_arg<float>(str);
}
//********************** END string to argument impls

template <typename... Args, int... Is>
/*constexpr*/ inline util::tuple<Args...> strs_to_args(util::array<const char*, sizeof...(Args)> args, util::index_sequence<Is...> idx)
{
    auto str = args;
    return {str_to_arg<Args>(args[Is])...}; // NOLINT
}

template <typename... Args, int... Is>
/*constexpr*/ inline void call_helper(util::function<void, Args...> fn, const util::tuple<Args...> arg_tuple, util::index_sequence<Is...> idx)
{
    fn(util::get<Is>(arg_tuple)...);
}

template <typename... Args>
/*constexpr*/ inline void call_with_str_args(util::function<void, Args...> fn, util::array<const char*, sizeof...(Args)> args)
{
    auto arg_tuple = strs_to_args<Args...>(args, util::make_index_sequence<sizeof...(Args)>());

    call_helper(fn, arg_tuple, util::make_index_sequence<sizeof...(Args)>());
}


template <typename... Args>
constexpr auto make_command_function(util::function<void, Args...> fn)
{
    constexpr auto caller_fn = call_with_str_args<Args...>;

    return [fn] (util::array<const char*, sizeof...(Args)> args) { caller_fn(fn, args); };
}

} // namespace detail


class CommandFn
{
public:
    template <typename... Args>
    constexpr CommandFn(util::function<void, Args...> fn)
        : fn_(detail::make_command_function(fn))
    {}

    template <int N>
    void call(util::array<const char*, N> args) const
    {
        fn_(args);
    }

private:
    static void foo(int, int) {}
    using fn_type = decltype(detail::make_command_function(foo));
    const  fn_type fn_;
};