#ifndef BESTGNSSPOSB_H
#define BESTGNSSPOSB_H

#include <functional>
#include <cstdint>
#include <cstring>

#include "novatel_binary_base.h"

namespace Novatel {
// BESTGNSSPOS binary message (ID = 1429).
// Binary data layout (72 bytes):
//   uint32_t solStatus       [0:4]
//   uint32_t posType         [4:8]
//   double   lat             [8:16]
//   double   lon             [16:24]
//   double   hgt             [24:32]
//   float    undulation      [32:36]
//   uint32_t datumId         [36:40]
//   float    latStd          [40:44]
//   float    lonStd          [44:48]
//   float    hgtStd          [48:52]
//   char     stnId[4]        [52:56]
//   float    diffAge         [56:60]
//   float    solAge          [60:64]
//   uint8_t  numSvs          [64]
//   uint8_t  numSolSvs       [65]
//   uint8_t  numGgL1         [66]
//   uint8_t  numSolMultiSvs  [67]
//   uint8_t  reserved        [68]
//   uint8_t  extSolStat      [69]
//   uint8_t  galBdsSigMask   [70]
//   uint8_t  gpsGloSigMask   [71]
struct BestGnssPosB {
    uint32_t solStatus;
    uint32_t posType;
    double   lat;
    double   lon;
    double   hgt;
    float    undulation;
    uint32_t datumId;
    float    latStd;
    float    lonStd;
    float    hgtStd;
    char     stnId[4];
    float    diffAge;
    float    solAge;
    uint8_t  numSvs;
    uint8_t  numSolSvs;
    uint8_t  numGgL1;
    uint8_t  numSolMultiSvs;
    uint8_t  extSolStat;
    uint8_t  galBdsSigMask;
    uint8_t  gpsGloSigMask;
    uint16_t gpsWeek;     // from header
    double   gpsSeconds;  // gpsMsec / 1000.0 from header
};

class BestGnssPosBParser {
public:
    using FrameCallback    = std::function<void(const BestGnssPosB &frame)>;
    using RawFrameCallback = std::function<void(const uint8_t *data, uint32_t len)>;

    static constexpr uint16_t MSG_ID   = 1429;
    static constexpr uint16_t DATA_LEN = 72;

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
    explicit BestGnssPosBParser(
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
                    BestGnssPosB frame{};
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

    bool ParseFrame(BestGnssPosB &f) {
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
        memcpy(&f.lat, d + 8, 8);
        memcpy(&f.lon, d + 16, 8);
        memcpy(&f.hgt, d + 24, 8);
        memcpy(&f.undulation, d + 32, 4);
        memcpy(&f.datumId, d + 36, 4);
        memcpy(&f.latStd, d + 40, 4);
        memcpy(&f.lonStd, d + 44, 4);
        memcpy(&f.hgtStd, d + 48, 4);
        memcpy(f.stnId, d + 52, 4);
        memcpy(&f.diffAge, d + 56, 4);
        memcpy(&f.solAge, d + 60, 4);
        f.numSvs         = d[64];
        f.numSolSvs      = d[65];
        f.numGgL1        = d[66];
        f.numSolMultiSvs = d[67];
        f.extSolStat     = d[69];
        f.galBdsSigMask  = d[70];
        f.gpsGloSigMask  = d[71];
        return true;
    }
};
}  // namespace Novatel

#endif  // BESTGNSSPOSB_H
