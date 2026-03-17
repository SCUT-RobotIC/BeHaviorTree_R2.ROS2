#ifndef STM32_COMM__UTILS_HPP_
#define STM32_COMM__UTILS_HPP_

#include "common_headers.hpp"
#include <cstring>

namespace utils {

template <typename T>
inline T combine_bytes(const uint8_t* data) {
    T result = 0;
    std::memcpy(&result, data, sizeof(T));
    return result;
}

} // namespace utils

#endif // STM32_COMM__UTILS_HPP_
