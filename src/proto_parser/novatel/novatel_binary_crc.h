#ifndef NOVATEL_BINARY_CRC_H
#define NOVATEL_BINARY_CRC_H

#include <cstdint>

namespace Novatel {
// CRC32 over raw bytes (same polynomial as ASCII CRC).
inline auto calculateBinaryCrc32(const uint8_t *data, uint32_t length) -> uint32_t {
    uint32_t crc = 0;
    for(uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(int j = 0; j < 8; j++) {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
        }
    }
    return crc;
}

// Validate binary frame CRC.
// buf: pointer to frame start (sync1 = 0xAA).
// frameLen: total bytes = header_len + msg_len + 4 (CRC).
inline auto validateBinaryCrc32(const uint8_t *buf, uint32_t frameLen) -> bool {
    if(frameLen < 4) {
        return false;
    }
    uint32_t dataLen = frameLen - 4;
    uint32_t calc    = calculateBinaryCrc32(buf, dataLen);
    uint32_t recv    = static_cast<uint32_t>(buf[dataLen])
                    | (static_cast<uint32_t>(buf[dataLen + 1]) << 8)
                    | (static_cast<uint32_t>(buf[dataLen + 2]) << 16)
                    | (static_cast<uint32_t>(buf[dataLen + 3]) << 24);
    return calc == recv;
}
}  // namespace Novatel

#endif  // NOVATEL_BINARY_CRC_H
