#pragma once

#include "test_utils.h"
#include <vector>
#include <cstdint>
#include <cstring>

// ── NovAtel binary frame builder ────────────────────────────────────────────
// Assembles a standard 28-byte long-header binary frame with correct CRC32.
// msgId    : NovAtel message ID (little-endian stored at offset 4-5)
// week     : GPS week (offset 14-15)
// msec     : GPS milliseconds (offset 16-19)
// data/len : message body bytes
static inline std::vector<uint8_t>
buildBinaryFrame(uint16_t msgId, uint16_t week, uint32_t msec,
                 const uint8_t *data, uint16_t dataLen)
{
    constexpr uint8_t HDR_LEN = 28;
    uint32_t totalLen = HDR_LEN + dataLen + 4;
    std::vector<uint8_t> f(totalLen, 0);

    // Sync + header-len
    f[0] = 0xAA; f[1] = 0x44; f[2] = 0x12; f[3] = HDR_LEN;
    // Message ID (LE)
    f[4] = msgId & 0xFF;          f[5] = (msgId >> 8) & 0xFF;
    // msg_type = 0 (binary), port_addr = 0
    f[6] = 0x00; f[7] = 0x00;
    // Message length (LE)
    f[8] = dataLen & 0xFF;        f[9] = (dataLen >> 8) & 0xFF;
    // Sequence = 0
    f[10] = 0; f[11] = 0;
    // Idle time = 0, Time status = 180 (FINESTEERING)
    f[12] = 0; f[13] = 180;
    // GPS week (LE)
    f[14] = week & 0xFF;          f[15] = (week >> 8) & 0xFF;
    // GPS milliseconds (LE)
    f[16] = msec & 0xFF;          f[17] = (msec >> 8)  & 0xFF;
    f[18] = (msec >> 16) & 0xFF;  f[19] = (msec >> 24) & 0xFF;
    // rx_status, reserved, sw_version already 0

    // Body
    if (data && dataLen > 0) memcpy(f.data() + HDR_LEN, data, dataLen);

    // CRC32 (same polynomial as NovAtel ASCII)
    uint32_t crc = 0;
    for (uint32_t i = 0; i < HDR_LEN + dataLen; i++) {
        crc ^= f[i];
        for (int j = 0; j < 8; j++) crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
    }
    f[HDR_LEN + dataLen + 0] = crc & 0xFF;
    f[HDR_LEN + dataLen + 1] = (crc >> 8)  & 0xFF;
    f[HDR_LEN + dataLen + 2] = (crc >> 16) & 0xFF;
    f[HDR_LEN + dataLen + 3] = (crc >> 24) & 0xFF;

    return f;
}

// Corrupt the last CRC byte so validation fails.
static inline std::vector<uint8_t> corruptCrc(std::vector<uint8_t> frame) {
    frame.back() ^= 0xFF;
    return frame;
}

// Little-endian write helpers
static inline void writeU32LE(uint8_t *p, uint32_t v) {
    p[0]=v&0xFF; p[1]=(v>>8)&0xFF; p[2]=(v>>16)&0xFF; p[3]=(v>>24)&0xFF;
}
static inline void writeU16LE(uint8_t *p, uint16_t v) {
    p[0]=v&0xFF; p[1]=(v>>8)&0xFF;
}
static inline void writeF32LE(uint8_t *p, float v) {
    memcpy(p, &v, 4);
}
static inline void writeF64LE(uint8_t *p, double v) {
    memcpy(p, &v, 8);
}
static inline void writeI32LE(uint8_t *p, int32_t v) {
    memcpy(p, &v, 4);
}
