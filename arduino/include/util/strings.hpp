#pragma once

#include "macros.hpp"
#ifdef ARDUINO
#include <WString.h>
#endif

namespace util {

constexpr bool is_whitespace(char chr)
{
    return chr == ' ' || chr == '\n' || chr == '\r';
}

class string_view
{
public:
    // A view on a string; not mutatable
    using const_iterator = const char*;
    using iterator = const_iterator;
    static constexpr auto npos = nullptr;

    constexpr string_view() = default;

    constexpr string_view(const char* str) // NOLINT
        : start_(str)
        , len_(static_cast<int>(len(str)))
    {}

    string_view(String str) // NOLINT
        : start_(str.c_str())
        , len_(static_cast<int>(str.length()))
    {}

    template <typename Iterator>
    constexpr string_view(Iterator first, Iterator last)
        : start_(first)
        , len_(last - first)
    {}

    [[nodiscard]] constexpr int size() const { return len_; }

    constexpr bool operator==(string_view rhs) const { return cmp(*this, rhs); }

#ifdef DEBUG
    // Expensive fn, copying the entire buffer
    // Should only be used for debug statements
    String printable() const { return String(start_).substring(0, len_); }
#endif

    // Iterator helper methods
    [[nodiscard]] constexpr const_iterator cbegin() const { return start_; }
    [[nodiscard]] constexpr const_iterator cend() const { return start_ + len_; }
    [[nodiscard]] constexpr const_iterator begin() const { return start_; }
    [[nodiscard]] constexpr const_iterator end() const { return start_ + len_; }

    // Utility functions
    constexpr const_iterator find(char chr, const_iterator start = npos) const
    {
        if (start == npos) {
            start = start_;
        }

        for (auto iter = start; iter != end(); ++iter) {
            if (*iter == chr) {
                return iter;
            }
        }
        return npos;
    }
    constexpr const_iterator find(char chr, unsigned start = 0) const
    {
        // Delegating call
        return find(chr, begin() + start);
    }

private:
    const char* start_{};
    int len_{};

    static constexpr int len(const char* str)
    {
        int len = 0;

        while (*str++ != '\0') {
            ++len;
        }

        return len;
    }

    // Compares two string_views for equality
    static constexpr bool cmp(string_view lhs, string_view rhs)
    {
        if (lhs.len_ != rhs.len_) {
            return false;
        }

        const char* lhs_ptr = lhs.start_;
        const char* rhs_ptr = rhs.start_;

        for (int i = 0; i < lhs.len_; ++i) {
            if (*lhs_ptr != *rhs_ptr) {
                return false;
            }
            ++lhs_ptr;
            ++rhs_ptr;
        }

        return true;
    }
};

} // namespace util