#ifndef INSSTDEVSB_H
#define INSSTDEVSB_H

#include <functional>
#include <cstdint>
#include <cstring>

#include "novatel_binary_base.h"

namespace Novatel {
// INSSTDEV binary message (ID = 2051).
// Binary data layout (52 bytes):
//   float    latSigma         [0:4]
//   float    lonSigma         [4:8]
//   float    hgtSigma         [8:12]
//   float    northVelSigma    [12:16]
//   float    eastVelSigma     [16:20]
//   float    upVelSigma       [20:24]
//   float    rollSigma        [24:28]
//   float    pitchSigma       [28:32]
//   float    azimuthSigma     [32:36]
//   uint32_t extSolStat       [36:40]
//   uint16_t timeSinceUpdate  [40:42]
//   uint16_t reserved1        [42:44]
//   uint32_t reserved2        [44:48]
//   uint32_t reserved3        [48:52]
struct InsStdevsB {
    uint16_t gpsWeek;
    double   gpsSeconds;
    float    latSigma;
    float    lonSigma;
    float    hgtSigma;
    float    northVelSigma;
    float    eastVelSigma;
    float    upVelSigma;
    float    rollSigma;
    float    pitchSigma;
    float    azimuthSigma;
    uint32_t extSolStat;
    uint16_t timeSinceUpdate;
    uint16_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
};

class InsStdevsBParser {
public:
    using FrameCallback    = std::function<void(const InsStdevsB &frame)>;
    using RawFrameCallback = std::function<void(const uint8_t *data, uint32_t len)>;

    static constexpr uint16_t MSG_ID   = 2051;
    static constexpr uint16_t DATA_LEN = 52;

private:
    enum class State : uint8_t {
        SYNC1,
        SYNC2,
        SYNC3,
        HDRLEN,
        HEADER,
        DATA,
        CRC
    };

    State            m_state;
    uint8_t          m_headerLen;
    uint16_t         m_msgLen;
    uint32_t         m_index;
    uint32_t         m_remain;
    uint8_t          m_frame[BINARY_MAX_FRAME];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit InsStdevsBParser(
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
                    InsStdevsB frame{};
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

    bool ParseFrame(InsStdevsB &f) {
        uint32_t totalLen = static_cast<uint32_t>(m_headerLen) + m_msgLen + 4;
        if(!validateBinaryCrc32(m_frame, totalLen)) {
            return false;
        }
        if(m_msgLen < DATA_LEN) {
            return false;
        }
        auto hdr         = parseBinaryHeader(m_frame);
        f.gpsWeek        = hdr.gpsWeek;
        f.gpsSeconds     = hdr.gpsMsec / 1000.0;
        const uint8_t *d = m_frame + m_headerLen;
        memcpy(&f.latSigma, d + 0, 4);
        memcpy(&f.lonSigma, d + 4, 4);
        memcpy(&f.hgtSigma, d + 8, 4);
        memcpy(&f.northVelSigma, d + 12, 4);
        memcpy(&f.eastVelSigma, d + 16, 4);
        memcpy(&f.upVelSigma, d + 20, 4);
        memcpy(&f.rollSigma, d + 24, 4);
        memcpy(&f.pitchSigma, d + 28, 4);
        memcpy(&f.azimuthSigma, d + 32, 4);
        memcpy(&f.extSolStat, d + 36, 4);
        memcpy(&f.timeSinceUpdate, d + 40, 2);
        memcpy(&f.reserved1, d + 42, 2);
        memcpy(&f.reserved2, d + 44, 4);
        memcpy(&f.reserved3, d + 48, 4);
        return true;
    }
};
}  // namespace Novatel

#endif  // INSSTDEVSB_H
