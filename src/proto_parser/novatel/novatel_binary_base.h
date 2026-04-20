/*
 * @Author: 熊昱卿(Aurson) jassimxiong@gmail.com
 * @Date: 2026-04-19 23:34:10
 * @LastEditors: 熊昱卿(Aurson) jassimxiong@gmail.com
 * @LastEditTime: 2026-04-21 01:09:20
 * @Description: 
 * Copyright (c) 2026 by Aurson, All Rights Reserved. 
 */
#ifndef NOVATEL_BINARY_BASE_H
#define NOVATEL_BINARY_BASE_H

#include <cstdint>
#include <cstring>

#include "novatel_binary_crc.h"

namespace Novatel {
static constexpr uint8_t  BINARY_SYNC1     = 0xAA;
static constexpr uint8_t  BINARY_SYNC2     = 0x44;
static constexpr uint8_t  BINARY_SYNC3     = 0x12;
static constexpr uint32_t BINARY_MAX_FRAME = 2048;

// Standard NovAtel binary long header (28 bytes).
struct BinaryHeader {
    uint8_t  headerLen;
    uint16_t msgId;
    uint8_t  msgType;
    uint8_t  portAddr;
    uint16_t msgLen;
    uint16_t sequence;
    uint8_t  idleTime;
    uint8_t  timeStatus;
    uint16_t gpsWeek;
    uint32_t gpsMsec;  // milliseconds from GPS week start
    uint32_t rxStatus;
    uint16_t swVersion;
};

// Parse the long header fields from a frame buffer starting at sync1.
inline BinaryHeader parseBinaryHeader(const uint8_t *buf) {
    BinaryHeader h{};
    h.headerLen  = buf[3];
    h.msgId      = static_cast<uint16_t>(buf[4] | (buf[5] << 8));
    h.msgType    = buf[6];
    h.portAddr   = buf[7];
    h.msgLen     = static_cast<uint16_t>(buf[8] | (buf[9] << 8));
    h.sequence   = static_cast<uint16_t>(buf[10] | (buf[11] << 8));
    h.idleTime   = buf[12];
    h.timeStatus = buf[13];
    h.gpsWeek    = static_cast<uint16_t>(buf[14] | (buf[15] << 8));
    h.gpsMsec    = static_cast<uint32_t>(buf[16] | (buf[17] << 8) | (buf[18] << 16) | (buf[19] << 24));
    h.rxStatus   = static_cast<uint32_t>(buf[20] | (buf[21] << 8) | (buf[22] << 16) | (buf[23] << 24));
    h.swVersion  = static_cast<uint16_t>(buf[26] | (buf[27] << 8));
    return h;
}
}  // namespace Novatel

#endif  // NOVATEL_BINARY_BASE_H
