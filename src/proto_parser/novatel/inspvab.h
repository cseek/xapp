#ifndef INSPVAB_H
#define INSPVAB_H

#include <functional>
#include <cstdint>
#include <cstring>

#include "novatel_binary_base.h"

namespace Novatel {
// INSPVA binary message (ID = 507).
// Binary data layout (88 bytes):
//   uint32_t week       [0:4]
//   double   seconds    [4:12]
//   double   lat        [12:20]
//   double   lon        [20:28]
//   double   height     [28:36]
//   double   northVel   [36:44]
//   double   eastVel    [44:52]
//   double   upVel      [52:60]
//   double   roll       [60:68]
//   double   pitch      [68:76]
//   double   azimuth    [76:84]
//   uint32_t status     [84:88]
struct InsPvaB {
    uint32_t gpsWeek;
    double   gpsSeconds;
    double   lat;
    double   lon;
    double   height;
    double   northVel;
    double   eastVel;
    double   upVel;
    double   roll;
    double   pitch;
    double   azimuth;
    uint32_t status;
};

class InsPvaBParser {
public:
    using FrameCallback    = std::function<void(const InsPvaB &frame)>;
    using RawFrameCallback = std::function<void(const uint8_t *data, uint32_t len)>;

    static constexpr uint16_t MSG_ID   = 507;
    static constexpr uint16_t DATA_LEN = 88;

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
    explicit InsPvaBParser(
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
                    InsPvaB frame{};
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

    bool ParseFrame(InsPvaB &f) {
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
        memcpy(&f.lat, d + 12, 8);
        memcpy(&f.lon, d + 20, 8);
        memcpy(&f.height, d + 28, 8);
        memcpy(&f.northVel, d + 36, 8);
        memcpy(&f.eastVel, d + 44, 8);
        memcpy(&f.upVel, d + 52, 8);
        memcpy(&f.roll, d + 60, 8);
        memcpy(&f.pitch, d + 68, 8);
        memcpy(&f.azimuth, d + 76, 8);
        memcpy(&f.status, d + 84, 4);
        f.gpsWeek = bodyWeek;
        return true;
    }
};
}  // namespace Novatel

#endif  // INSPVAB_H
