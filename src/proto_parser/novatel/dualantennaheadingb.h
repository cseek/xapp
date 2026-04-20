#ifndef DUALANTENNAHEADINGB_H
#define DUALANTENNAHEADINGB_H

#include <functional>
#include <cstdint>
#include <cstring>

#include "novatel_binary_base.h"

namespace Novatel {
// DUALANTENNAHEADING binary message (ID = 971).
// Binary data layout (44 bytes):
//   uint32_t solStatus       [0:4]
//   uint32_t posType         [4:8]
//   float    length          [8:12]
//   float    heading         [12:16]
//   float    pitch           [16:20]
//   float    reserved        [20:24]
//   float    hdgStdDev       [24:28]
//   float    ptchStdDev      [28:32]
//   char     stnId[4]        [32:36]
//   uint8_t  numSvs          [36]
//   uint8_t  numSolnSvs      [37]
//   uint8_t  numObs          [38]
//   uint8_t  numMulti        [39]
//   uint8_t  solSource       [40]
//   uint8_t  extSolStat      [41]
//   uint8_t  galBdsSigMask   [42]
//   uint8_t  gpsGloSigMask   [43]
struct DualAntennaHeadingB {
    uint32_t solStatus;
    uint32_t posType;
    float    length;
    float    heading;
    float    pitch;
    float    hdgStdDev;
    float    ptchStdDev;
    char     stnId[4];
    uint8_t  numSvs;
    uint8_t  numSolnSvs;
    uint8_t  numObs;
    uint8_t  numMulti;
    uint8_t  solSource;
    uint8_t  extSolStat;
    uint8_t  galBdsSigMask;
    uint8_t  gpsGloSigMask;
    uint16_t gpsWeek;
    double   gpsSeconds;
};

class DualAntennaHeadingBParser {
public:
    using FrameCallback    = std::function<void(const DualAntennaHeadingB &frame)>;
    using RawFrameCallback = std::function<void(const uint8_t *data, uint32_t len)>;

    static constexpr uint16_t MSG_ID   = 971;
    static constexpr uint16_t DATA_LEN = 44;

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
    explicit DualAntennaHeadingBParser(
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
                    DualAntennaHeadingB frame{};
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

    bool ParseFrame(DualAntennaHeadingB &f) {
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
        memcpy(&f.solStatus, d + 0, 4);
        memcpy(&f.posType, d + 4, 4);
        memcpy(&f.length, d + 8, 4);
        memcpy(&f.heading, d + 12, 4);
        memcpy(&f.pitch, d + 16, 4);
        // d[20:24] reserved float - skip
        memcpy(&f.hdgStdDev, d + 24, 4);
        memcpy(&f.ptchStdDev, d + 28, 4);
        memcpy(f.stnId, d + 32, 4);
        f.numSvs        = d[36];
        f.numSolnSvs    = d[37];
        f.numObs        = d[38];
        f.numMulti      = d[39];
        f.solSource     = d[40];
        f.extSolStat    = d[41];
        f.galBdsSigMask = d[42];
        f.gpsGloSigMask = d[43];
        return true;
    }
};
}  // namespace Novatel

#endif  // DUALANTENNAHEADINGB_H
