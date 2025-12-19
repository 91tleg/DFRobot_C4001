#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>

template<typename T>
concept UartTransport =
requires(T t,
         const uint8_t *out,
         uint8_t *in,
         size_t len,
         uint32_t timeout)
{
    { t.write(out, len) } -> std::convertible_to<int>;
    { t.read(in, len, timeout) } -> std::convertible_to<int>;
    { t.flush() };
};
