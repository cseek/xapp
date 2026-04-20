#ifndef NOVATEL_ASCII_CRC_H
#define NOVATEL_ASCII_CRC_H

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cctype>

namespace Novatel {
inline auto calculateAsciiCrc32(const char *data, uint32_t length) -> uint32_t {
    uint32_t crc = 0;
    for(uint32_t i = 0; i < length; i++) {
        crc ^= static_cast<uint8_t>(data[i]);
        for(int j = 0; j < 8; j++) {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
        }
    }
    return crc;
}

inline auto validateAsciiFrameCrc32(char *buf, char startChar) -> bool {
    char *start = strchr(buf, startChar);
    char *star  = strchr(buf, '*');
    if(start == nullptr || star == nullptr || star <= start + 1) {
        return false;
    }

    char *crcToken = star + 1;
    int   crcLen   = 0;
    while(std::isxdigit(static_cast<unsigned char>(crcToken[crcLen]))) {
        crcLen++;
    }
    if(crcLen != 8) {
        return false;
    }

    char saved       = crcToken[8];
    crcToken[8]      = '\0';
    uint32_t recvCrc = static_cast<uint32_t>(strtoul(crcToken, nullptr, 16));
    crcToken[8]      = saved;

    uint32_t calcCrc = calculateAsciiCrc32(start + 1, static_cast<uint32_t>(star - (start + 1)));
    return recvCrc == calcCrc;
}
}  // namespace Novatel

#endif  // NOVATEL_ASCII_CRC_H
