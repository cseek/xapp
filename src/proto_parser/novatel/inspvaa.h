#ifndef INSPVAA_H
#define INSPVAA_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "novatel_ascii_crc.h"

namespace Novatel {
// #INSPVAA,<port>,<seq>,<idle%>,<time_status>,<week>,<sec>,<rx_status>,<hdr_len>,<msg_len>;
// <week>,<seconds>,<lat>,<lon>,<height>,
// <north_vel>,<east_vel>,<up_vel>,
// <roll>,<pitch>,<azimuth>,<status>*<CRC32>
struct InsPvaA {
    uint32_t gpsWeek;     // GNSS week
    double   gpsSeconds;  // seconds from week start
    double   lat;         // latitude (degrees)
    double   lon;         // longitude (degrees)
    double   height;      // ellipsoidal height (m)
    double   northVel;    // north velocity (m/s)
    double   eastVel;     // east velocity (m/s)
    double   upVel;       // up velocity (m/s)
    double   roll;        // roll (degrees)
    double   pitch;       // pitch (degrees)
    double   azimuth;     // azimuth clockwise from North (degrees)
    char     status[32];  // INS status string
};

class InsPvaAParser {
public:
    using FrameCallback    = std::function<void(const InsPvaA &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    // match: #INSPVAA  (7 chars after #)
    enum class State : uint8_t {
        SHARP,
        H1,  // I
        H2,  // N
        H3,  // S
        H4,  // P
        H5,  // V
        H6,  // A
        H7,  // A
        CR,
        LF
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[512];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit InsPvaAParser(
        const FrameCallback    &frameCallback,
        const RawFrameCallback &rawFrameCallback)
        : m_state(State::SHARP),
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
        case State::SHARP:
            Select(byte, '#', State::H1);
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
            Select(byte, 'P', State::H5);
            break;
        case State::H5:
            Select(byte, 'V', State::H6);
            break;
        case State::H6:
            Select(byte, 'A', State::H7);
            break;
        case State::H7:
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
                    InsPvaA frame{};
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
        m_state = State::SHARP;
    }

    inline auto ParserFrame(InsPvaA &f) -> bool {
        f = InsPvaA{};
        char     buf[512];
        uint32_t len = m_index + 1;
        memcpy(buf, m_frame, len);
        buf[len] = '\0';

        if(!validateAsciiFrameCrc32(buf, '#')) {
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
            // ── header 0-9 ──────────────────────────────────────────────
            case 0:
                break;  // #INSPVAA
            case 1:
                break;  // port
            case 2:
                break;  // sequence
            case 3:
                break;  // idle%
            case 4:
                break;  // time_status
            case 5:
                break;  // GPS week (header)
            case 6:
                break;  // GPS seconds (header)
            case 7:
                break;  // rx_status
            case 8:
                break;  // hdr_len
            case 9:
                break;  // msg_len
            // ── body 10-21 ──────────────────────────────────────────────
            case 10:
                f.gpsWeek = static_cast<uint32_t>(strtoul(token, nullptr, 10));
                break;
            case 11:
                f.gpsSeconds = strtod(token, nullptr);
                break;
            case 12:
                f.lat = strtod(token, nullptr);
                break;
            case 13:
                f.lon = strtod(token, nullptr);
                break;
            case 14:
                f.height = strtod(token, nullptr);
                break;
            case 15:
                f.northVel = strtod(token, nullptr);
                break;
            case 16:
                f.eastVel = strtod(token, nullptr);
                break;
            case 17:
                f.upVel = strtod(token, nullptr);
                break;
            case 18:
                f.roll = strtod(token, nullptr);
                break;
            case 19:
                f.pitch = strtod(token, nullptr);
                break;
            case 20:
                f.azimuth = strtod(token, nullptr);
                break;
            case 21:
                strncpy(f.status, token, sizeof(f.status) - 1);
                break;
            default:
                break;
            }
            field++;
        }
        return field >= 22;
    }
};
}  // namespace Novatel

#endif  // INSPVAA_H
