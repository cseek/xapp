#ifndef RMC_H
#define RMC_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "nmea_checksum.h"

namespace Nmea {
// $XXRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a[,mode]*hh\r\n
// fields:
//  0  $XXRMC          talker ID
//  1  hhmmss.ss       UTC time
//  2  A/V             status  (A=active, V=void)
//  3  llll.ll         latitude  DDMM.MMMMM
//  4  N/S             lat direction
//  5  yyyyy.yy        longitude DDDMM.MMMMM
//  6  E/W             lon direction
//  7  x.x             speed over ground (knots)
//  8  x.x             course over ground (degrees, true)
//  9  ddmmyy          date
// 10  x.x             magnetic variation (degrees, optional)
// 11  E/W             magnetic variation direction (optional)
// 12  A/D/E/N         mode indicator (NMEA 2.3+, optional)
struct Rmc {
    char   talkerId[3];    // talker ID, e.g., "GP"
    double utcTime;        // UTC time, hhmmss.ss
    char   status;         // A=active/valid, V=void/invalid
    double latitude;       // DDMM.MMMMM
    char   latDir;         // N or S
    double longitude;      // DDDMM.MMMMM
    char   lonDir;         // E or W
    double speedKnots;     // speed over ground (knots)
    double courseDeg;      // course over ground (degrees true)
    int    date;           // ddmmyy as integer
    double magVariation;   // magnetic variation (degrees, 0 if absent)
    char   magVarDir;      // E or W ('\0' if absent)
    char   modeIndicator;  // A/D/E/N, '\0' if absent
};

class RmcParser {
public:
    using FrameCallback    = std::function<void(const Rmc &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    enum class State : uint8_t {
        DOLLAR,     // $
        TALKER1,    // first uppercase letter of talker ID
        TALKER2,    // second uppercase letter of talker ID
        SENTENCE1,  // R
        SENTENCE2,  // M
        SENTENCE3,  // C
        COMMA,      // ,
        DATA,       // payload until \r
        LF          // \n
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[1024];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit RmcParser(
        const FrameCallback    &frameCallback,
        const RawFrameCallback &rawFrameCallback)
        : m_state(State::DOLLAR),
          m_index(0),
          m_frameCallback(frameCallback),
          m_rawFrameCallback(rawFrameCallback) {
        memset(&m_frame, 0, sizeof(m_frame));
    }

    void Parse(const uint8_t *data, uint32_t size) {
        for(uint32_t i = 0; i < size; i++) {
            ParseByteStream(data[i]);
        }
    }

    void ParseByteStream(uint8_t byte) {
        switch(m_state) {
        case State::DOLLAR:
            Select(byte, '$', State::TALKER1);
            break;
        case State::TALKER1:
            if(byte >= 'A' && byte <= 'Z') {
                m_frame[m_index++] = byte;
                m_state            = State::TALKER2;
            } else {
                Reset();
            }
            break;
        case State::TALKER2:
            if(byte >= 'A' && byte <= 'Z') {
                m_frame[m_index++] = byte;
                m_state            = State::SENTENCE1;
            } else {
                Reset();
            }
            break;
        case State::SENTENCE1:
            Select(byte, 'R', State::SENTENCE2);
            break;
        case State::SENTENCE2:
            Select(byte, 'M', State::SENTENCE3);
            break;
        case State::SENTENCE3:
            Select(byte, 'C', State::COMMA);
            break;
        case State::COMMA:
            Select(byte, ',', State::DATA);
            break;
        case State::DATA:
            m_frame[m_index++] = byte;
            if(m_index > sizeof(m_frame) - 2 || m_index > MAX_NMEA_SENTENCE_LEN - 1) {
                Reset();
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
                    Rmc frame{};
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
        if(byte == static_cast<uint8_t>(cond)) {
            m_frame[m_index++] = byte;
            m_state            = nextState;
        } else {
            Reset();
        }
    }

    inline void Reset() {
        m_index = 0;
        m_state = State::DOLLAR;
    }

    inline bool ParserFrame(Rmc &f) {
        f = Rmc{};
        char     buf[1024];
        uint32_t len = m_index + 1;
        memcpy(buf, m_frame, len);
        buf[len] = '\0';

        if(!validateNmeaChecksum(buf)) {
            return false;
        }

        // Pointer-based split — preserves empty fields (strtok collapses them).
        char *p     = buf;
        int   field = 0;
        while(p != nullptr) {
            char *sep = p;
            while(*sep != '\0' && *sep != ',' && *sep != '\r' && *sep != '\n') {
                sep++;
            }
            char endChar = *sep;
            *sep         = '\0';
            char *token  = p;
            p            = (endChar == ',') ? sep + 1 : nullptr;

            // strip checksum (*hh)
            char *star = strchr(token, '*');
            if(star != nullptr) {
                *star = '\0';
            }

            switch(field) {
            case 0:  // $XXRMC — talker ID at [1..2]
                if(strlen(token) >= 3) {
                    f.talkerId[0] = token[1];
                    f.talkerId[1] = token[2];
                    f.talkerId[2] = '\0';
                }
                break;
            case 1:
                f.utcTime = strtod(token, nullptr);
                break;
            case 2:
                f.status = token[0];
                break;
            case 3:
                f.latitude = strtod(token, nullptr);
                break;
            case 4:
                f.latDir = token[0];
                break;
            case 5:
                f.longitude = strtod(token, nullptr);
                break;
            case 6:
                f.lonDir = token[0];
                break;
            case 7:
                f.speedKnots = strtod(token, nullptr);
                break;
            case 8:
                f.courseDeg = strtod(token, nullptr);
                break;
            case 9:
                f.date = static_cast<int>(strtol(token, nullptr, 10));
                break;
            case 10:
                f.magVariation = strtod(token, nullptr);
                break;
            case 11:
                f.magVarDir = token[0];
                break;
            case 12:
                f.modeIndicator = token[0];
                break;
            default:
                break;
            }
            field++;
        }
        return field >= 10;
    }
};
}  // namespace Nmea

#endif  // RMC_H
