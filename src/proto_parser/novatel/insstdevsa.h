#ifndef INSSTDEVSA_H
#define INSSTDEVSA_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "novatel_ascii_crc.h"

namespace Novatel {
// %INSSTDEVSA,<week>,<sec>;
// <lat_sigma>,<lon_sigma>,<hgt_sigma>,
// <north_vel_sigma>,<east_vel_sigma>,<up_vel_sigma>,
// <roll_sigma>,<pitch_sigma>,<azimuth_sigma>,
// <ext_sol_stat>,<time_since_update>,<reserved1>,<reserved2>,<reserved3>*<CRC32>
struct InsStdevsA {
    uint32_t gpsWeek;          // GNSS week (short header)
    double   gpsSeconds;       // seconds from week start (short header)
    float    latSigma;         // latitude sigma (m)
    float    lonSigma;         // longitude sigma (m)
    float    hgtSigma;         // height sigma (m)
    float    northVelSigma;    // north velocity sigma (m/s)
    float    eastVelSigma;     // east velocity sigma (m/s)
    float    upVelSigma;       // up velocity sigma (m/s)
    float    rollSigma;        // roll sigma (deg)
    float    pitchSigma;       // pitch sigma (deg)
    float    azimuthSigma;     // azimuth sigma (deg)
    uint32_t extSolStat;       // extended solution status
    uint16_t timeSinceUpdate;  // elapsed time since update (s)
    uint16_t reserved1;        // reserved
    uint32_t reserved2;        // reserved
    uint32_t reserved3;        // reserved
};

class InsStdevsAParser {
public:
    using FrameCallback    = std::function<void(const InsStdevsA &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    // match: %INSSTDEVSA (10 chars after %)
    enum class State : uint8_t {
        PERCENT,
        H1,   // I
        H2,   // N
        H3,   // S
        H4,   // S
        H5,   // T
        H6,   // D
        H7,   // E
        H8,   // V
        H9,   // S
        H10,  // A
        CR,
        LF
    };

    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[512];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit InsStdevsAParser(
        const FrameCallback    &frameCallback,
        const RawFrameCallback &rawFrameCallback)
        : m_state(State::PERCENT),
          m_index(0),
          m_frameCallback(frameCallback),
          m_rawFrameCallback(rawFrameCallback) {
        memset(&m_frame, 0, sizeof(m_frame));
    }

public:
    void Parse(const uint8_t *data, uint32_t size) {
        for(uint32_t i = 0; i < size; i++) {
            ParseByteStream(data[i]);
        }
    }

    void ParseByteStream(uint8_t byte) {
        switch(m_state) {
        case State::PERCENT:
            Select(byte, '%', State::H1);
            break;
        case State::H1:
            Select(byte, 'I', State::H2);
            break;
        case State::H2:
            Select(byte, 'N', State::H3);
            break;
        case State::H3:
            Select(byte, 'S', State::H4);
            break;
        case State::H4:
            Select(byte, 'S', State::H5);
            break;
        case State::H5:
            Select(byte, 'T', State::H6);
            break;
        case State::H6:
            Select(byte, 'D', State::H7);
            break;
        case State::H7:
            Select(byte, 'E', State::H8);
            break;
        case State::H8:
            Select(byte, 'V', State::H9);
            break;
        case State::H9:
            Select(byte, 'S', State::H10);
            break;
        case State::H10:
            Select(byte, 'A', State::CR);
            break;

        case State::CR:
            m_frame[m_index++] = byte;
            if(m_index > sizeof(m_frame) - 2) {
                Reset();
                break;
            }
            if(byte == '\r') {
                m_state = State::LF;
            }
            break;

        case State::LF:
            m_frame[m_index] = byte;
            if(byte == '\n') {
                if(m_rawFrameCallback) {
                    m_rawFrameCallback(m_frame, m_index + 1);
                }
                if(m_frameCallback) {
                    InsStdevsA frame{};
                    if(ParserFrame(frame)) {
                        m_frameCallback(frame);
                    }
                }
            }
            Reset();
            break;

        default:
            Reset();
            break;
        }
    }

private:
    inline void Select(uint8_t byte, char cond, State nextState) {
        if(byte == cond) {
            m_frame[m_index++] = byte;
            m_state            = nextState;
        } else {
            Reset();
        }
    }

    inline void Reset() {
        m_index = 0;
        m_state = State::PERCENT;
    }

    inline auto ParserFrame(InsStdevsA &f) -> bool {
        f = InsStdevsA{};
        char     buf[512];
        uint32_t len = m_index + 1;  // include '\n'
        memcpy(buf, m_frame, len);
        buf[len] = '\0';

        if(!validateAsciiFrameCrc32(buf, '%')) {
            return false;
        }

        char *p     = buf;
        int   field = 0;
        while(p != nullptr) {
            char *sep = p;
            while(*sep != '\0' && *sep != ',' && *sep != ';' && *sep != '*' && *sep != '\r' && *sep != '\n') {
                sep++;
            }
            char endChar = *sep;
            *sep         = '\0';
            char *token  = p;
            p            = (endChar == ',' || endChar == ';' || endChar == '*') ? sep + 1 : nullptr;

            switch(field) {
            case 0:
                break;  // %INSSTDEVSA
            case 1:
                f.gpsWeek = static_cast<uint32_t>(strtoul(token, nullptr, 10));
                break;
            case 2:
                f.gpsSeconds = strtod(token, nullptr);
                break;
            case 3:
                f.latSigma = strtof(token, nullptr);
                break;
            case 4:
                f.lonSigma = strtof(token, nullptr);
                break;
            case 5:
                f.hgtSigma = strtof(token, nullptr);
                break;
            case 6:
                f.northVelSigma = strtof(token, nullptr);
                break;
            case 7:
                f.eastVelSigma = strtof(token, nullptr);
                break;
            case 8:
                f.upVelSigma = strtof(token, nullptr);
                break;
            case 9:
                f.rollSigma = strtof(token, nullptr);
                break;
            case 10:
                f.pitchSigma = strtof(token, nullptr);
                break;
            case 11:
                f.azimuthSigma = strtof(token, nullptr);
                break;
            case 12:
                f.extSolStat = static_cast<uint32_t>(strtoul(token, nullptr, 16));
                break;
            case 13:
                f.timeSinceUpdate = static_cast<uint16_t>(strtoul(token, nullptr, 10));
                break;
            case 14:
                f.reserved1 = static_cast<uint16_t>(strtoul(token, nullptr, 10));
                break;
            case 15:
                f.reserved2 = static_cast<uint32_t>(strtoul(token, nullptr, 16));
                break;
            case 16:
                f.reserved3 = static_cast<uint32_t>(strtoul(token, nullptr, 10));
                break;
            default:
                break;
            }
            field++;
        }
        return field >= 17;
    }
};
}  // namespace Novatel

#endif  // INSSTDEVSA_H
