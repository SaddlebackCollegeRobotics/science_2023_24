#pragma once

#ifdef ARDUINO
#include <WString.h>
#endif

namespace util {

constexpr int strlen(const char* str)
{
    int len = 0;

    while (*str++ != '\0') {
        ++len;
    }

    return len;
}

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

    constexpr string_view(const char* str) // NOLINT
        : start_(str)
        , len_(static_cast<int>(strlen(str)))
    {}

    template <typename Iterator>
    constexpr string_view(Iterator first, Iterator last)
        : start_(first)
        , len_(last - first - 1)
    {}

    [[nodiscard]] constexpr int size() const { return len_; }

    // Iterator helper methods
    [[nodiscard]] constexpr const_iterator cbegin() const { return start_; }
    [[nodiscard]] constexpr const_iterator cend() const { return start_ + len_; }
    [[nodiscard]] constexpr iterator begin() const { return start_; }
    [[nodiscard]] constexpr const_iterator end() const { return start_ + len_; }

private:
    const char* start_;
    int len_;
};

} // namespace util