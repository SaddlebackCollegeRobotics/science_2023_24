#pragma once

#include "array.hpp"
#include "tuple.hpp"

namespace util {

template <typename T>
struct equals
{
    constexpr bool operator()(const T& lhs, const T& rhs) const
    {
        return lhs == rhs;
    }
};

// Specializations for string comparisons
template <>
struct equals<const char*>
{
    // Compares two null-terminated strings for equality
    constexpr bool operator()(const char* lhs, const char* rhs) const
    {
        do {
            if (*lhs != *rhs) {
                return false;
            }

        } while (*lhs++ != '\0' && *rhs++ != 0);

        return true;
    }
};

template <>
struct equals<char*> : equals<const char*> {};

// template <typename T>
// class function;

// template <typename Ret, typename... Args>
// class function<Ret (Args...)>
// {
// public:
//     using return_t = Ret;
//     using arguments_t = tuple<Args...>;
//     using underlying_t = Ret (*)(Args...);

//     constexpr /*implicit*/ function(underlying_t fn)
//         : func_ptr_(fn)
//     {}

//     constexpr Ret operator()(Args... args)
//     {
//         return func_ptr_(args...);
//     }

//     constexpr Ret operator()(Args... args) const
//     {
//         return func_ptr_(args...);
//     }
// private:
//     underlying_t func_ptr_;
// };

template <typename Ret, typename... Args>
using function = Ret (*)(Args...);


} // namespace util