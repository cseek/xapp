#ifndef NMEA_CHECKSUM_H
#define NMEA_CHECKSUM_H

#include <cstdint>
#include <cctype>
#include <cstring>

namespace Nmea {
constexpr uint32_t MAX_NMEA_SENTENCE_LEN = 256;

inline auto calculateNmeaChecksum(const char *data, uint32_t length) -> uint8_t {
    uint8_t cs = 0;
    for(uint32_t i = 0; i < length; i++) {
        cs ^= static_cast<uint8_t>(data[i]);
    }
    return cs;
}

inline auto validateNmeaChecksum(const char *buf) -> bool {
    const char *dollar = strchr(buf, '$');
    const char *star   = strchr(buf, '*');
    if(dollar == nullptr || star == nullptr || star <= dollar + 1) {
        return false;
    }

    if(!std::isxdigit(static_cast<unsigned char>(star[1])) || !std::isxdigit(static_cast<unsigned char>(star[2]))) {
        return false;
    }

    auto hex4 = [](char c) -> uint8_t {
        if(c >= '0' && c <= '9') {
            return static_cast<uint8_t>(c - '0');
        }
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return static_cast<uint8_t>(10 + (c - 'A'));
    };

    uint8_t recv = static_cast<uint8_t>((hex4(star[1]) << 4) | hex4(star[2]));
    uint8_t calc = calculateNmeaChecksum(dollar + 1, static_cast<uint32_t>(star - (dollar + 1)));
    return recv == calc;
}
}  // namespace Nmea

#endif  // NMEA_CHECKSUM_H
