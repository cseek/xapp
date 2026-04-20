#ifndef RAWIMUB_H
#define RAWIMUB_H

#include <functional>
#include <cstdint>
#include <cstring>

#include "novatel_binary_base.h"

namespace Novatel {
// RAWIMU binary message (ID = 268).
// Binary data layout (40 bytes):
//   uint32_t week       [0:4]
//   double   seconds    [4:12]
//   uint32_t imuStatus  [12:16]
//   int32_t  zAccel     [16:20]
//   int32_t  negYAccel  [20:24]
//   int32_t  xAccel     [24:28]
//   int32_t  zGyro      [28:32]
//   int32_t  negYGyro   [32:36]
//   int32_t  xGyro      [36:40]
struct RawImuB {
    uint32_t gpsWeek;
    double   gpsSeconds;
    uint32_t imuStatus;
    int32_t  zAccel;
    int32_t  negYAccel;
    int32_t  xAccel;
    int32_t  zGyro;
    int32_t  negYGyro;
    int32_t  xGyro;
};

class RawImuBParser {
public:
    using FrameCallback    = std::function<void(const RawImuB &)>;
    using RawFrameCallback = std::function<void(const uint8_t *data, uint32_t len)>;

    static constexpr uint16_t MSG_ID   = 268;
    static constexpr uint16_t DATA_LEN = 40;

private:
    enum class State : uint8_t { SYNC1,
                                 SYNC2,
                                 SYNC3,
                                 HDRLEN,
                                 HEADER,
                                 DATA,
                                 CRC };

    State            m_state;
    uint8_t          m_headerLen;
    uint16_t         m_msgLen;
    uint32_t         m_index;
    uint32_t         m_remain;
    uint8_t          m_frame[BINARY_MAX_FRAME];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit RawImuBParser(
        const FrameCallback    &frameCallback,
        const RawFrameCallback &rawFrameCallback)
        : m_state(State::SYNC1),
          m_headerLen(0),
          m_msgLen(0),
          m_index(0),
          m_remain(0),
          m_frameCallback(frameCallback),
          m_rawFrameCallback(rawFrameCallback) {
        memset(m_frame, 0, sizeof(m_frame));
    }

    void Parse(const uint8_t *data, uint32_t size) {
        for(uint32_t i = 0; i < size; i++) {
            ParseByteStream(data[i]);
        }
    }

    void ParseByteStream(uint8_t byte) {
        switch(m_state) {
        case State::SYNC1:
            if(byte == BINARY_SYNC1) {
                m_index            = 0;
                m_frame[m_index++] = byte;
                m_state            = State::SYNC2;
            }
            break;
        case State::SYNC2:
            if(byte == BINARY_SYNC2) {
                m_frame[m_index++] = byte;
                m_state            = State::SYNC3;
            } else {
                Reset();
            }
            break;
        case State::SYNC3:
            if(byte == BINARY_SYNC3) {
                m_frame[m_index++] = byte;
                m_state            = State::HDRLEN;
            } else {
                Reset();
            }
            break;
        case State::HDRLEN:
            m_headerLen        = byte;
            m_frame[m_index++] = byte;
            m_remain           = m_headerLen - 4;
            m_state            = State::HEADER;
            break;
        case State::HEADER:
            m_frame[m_index++] = byte;
            if(--m_remain == 0) {
                uint16_t msgId = static_cast<uint16_t>(m_frame[4] | (m_frame[5] << 8));
                if(msgId != MSG_ID) {
                    Reset();
                    break;
                }
                m_msgLen = static_cast<uint16_t>(m_frame[8] | (m_frame[9] << 8));
                if(static_cast<uint32_t>(m_headerLen) + m_msgLen + 4 > BINARY_MAX_FRAME) {
                    Reset();
                    break;
                }
                m_remain = m_msgLen;
                m_state  = (m_msgLen > 0) ? State::DATA : State::CRC;
                if(m_msgLen == 0) {
                    m_remain = 4;
                }
            }
            break;
        case State::DATA:
            m_frame[m_index++] = byte;
            if(--m_remain == 0) {
                m_remain = 4;
                m_state  = State::CRC;
            }
            break;
        case State::CRC:
            m_frame[m_index++] = byte;
            if(--m_remain == 0) {
                uint32_t totalLen = m_index;
                if(m_rawFrameCallback) {
                    m_rawFrameCallback(m_frame, totalLen);
                }
                if(m_frameCallback) {
                    RawImuB frame{};
                    if(ParseFrame(frame)) {
                        m_frameCallback(frame);
                    }
                }
                Reset();
            }
            break;
        default:
            Reset();
            break;
        }
    }

private:
    void Reset() {
        m_state = State::SYNC1;
        m_index = 0;
    }

    bool ParseFrame(RawImuB &f) {
        uint32_t totalLen = static_cast<uint32_t>(m_headerLen) + m_msgLen + 4;
        if(!validateBinaryCrc32(m_frame, totalLen)) {
            return false;
        }
        if(m_msgLen < DATA_LEN) {
            return false;
        }
        const uint8_t *d = m_frame + m_headerLen;
        uint32_t       bodyWeek;
        memcpy(&bodyWeek, d + 0, 4);
        memcpy(&f.gpsSeconds, d + 4, 8);
        memcpy(&f.imuStatus, d + 12, 4);
        memcpy(&f.zAccel, d + 16, 4);
        memcpy(&f.negYAccel, d + 20, 4);
        memcpy(&f.xAccel, d + 24, 4);
        memcpy(&f.zGyro, d + 28, 4);
        memcpy(&f.negYGyro, d + 32, 4);
        memcpy(&f.xGyro, d + 36, 4);
        f.gpsWeek = bodyWeek;
        return true;
    }
};
}  // namespace Novatel

#endif  // RAWIMUB_H
