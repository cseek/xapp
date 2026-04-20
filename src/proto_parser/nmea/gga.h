#ifndef GGA_H
#define GGA_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "nmea_checksum.h"

namespace Nmea {
// $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh\r\n
struct Gga {
    char   talkerId[6];    // talker ID, e.g., "GP"
    double utcTime;        // UTC time, hhmmss.ss
    double latitude;       // latitude, DDMM.MMMMM
    char   latDir;         // N or S
    double longitude;      // longitude, DDDMM.MMMMM
    char   lonDir;         // E or W
    int    quality;        // GPS quality: 0=invalid, 1=GPS, 2=DGPS, ...
    int    numSatellites;  // number of satellites in use
    double hdop;           // horizontal dilution of precision
    double altitude;       // antenna altitude above sea level (meters)
    double geoidSep;       // geoidal separation (meters)
    double dgpsAge;        // age of DGPS data (seconds)
    int    dgpsStationId;  // DGPS reference station ID
};

class GgaParser {
public:
    using FrameCallback    = std::function<void(const Gga &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    enum class State : uint8_t {
        DOLLAR,     // $
        TALKER1,    // first char of talker ID (any uppercase letter)
        TALKER2,    // second char of talker ID (any uppercase letter)
        SENTENCE1,  // G
        SENTENCE2,  // G
        SENTENCE3,  // A
        COMMA,      // ,
        DATA,       // data payload until \r
        LF          // \n
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[1024];  // $GPGGA,...*hh\r\n
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit GgaParser(
        const FrameCallback    &frameCallback,
        const RawFrameCallback &rawFrameCallback)
        : m_state(State::DOLLAR),
          m_index(0),
          m_frameCallback(frameCallback),
          m_rawFrameCallback(rawFrameCallback) {
        memset(&m_frame, 0, sizeof(m_frame));
    }

public:
    void Parse(const uint8_t *data, uint32_t size) {
        for(auto i = 0; i < size; i++) {
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
            Select(byte, 'G', State::SENTENCE2);
            break;

        case State::SENTENCE2:
            Select(byte, 'G', State::SENTENCE3);
            break;

        case State::SENTENCE3:
            Select(byte, 'A', State::COMMA);
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
                    Gga frame{};
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
        m_state = State::DOLLAR;
    }

    inline auto ParserFrame(Gga &f) -> bool {
        f = Gga{};
        char     buf[1024];
        uint32_t len = m_index + 1;  // include '\n'
        memcpy(buf, m_frame, len);
        buf[len] = '\0';

        if(!validateNmeaChecksum(buf)) {
            return false;
        }

        // Use pointer-based split to preserve empty fields (strtok collapses them).
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

            // strip trailing '*hh' checksum
            char *star = strchr(token, '*');
            if(star != nullptr) {
                *star = '\0';
            }

            switch(field) {
            case 0:  // $XXGGA — extract talker ID from chars [1..2]
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
                f.latitude = strtod(token, nullptr);
                break;
            case 3:
                f.latDir = token[0];
                break;
            case 4:
                f.longitude = strtod(token, nullptr);
                break;
            case 5:
                f.lonDir = token[0];
                break;
            case 6:
                f.quality = static_cast<int>(strtol(token, nullptr, 10));
                break;
            case 7:
                f.numSatellites = static_cast<int>(strtol(token, nullptr, 10));
                break;
            case 8:
                f.hdop = strtod(token, nullptr);
                break;
            case 9:
                f.altitude = strtod(token, nullptr);
                break;
            case 10: /* altitude unit M — skip */
                break;
            case 11:
                f.geoidSep = strtod(token, nullptr);
                break;
            case 12: /* geoid sep unit M — skip */
                break;
            case 13:
                f.dgpsAge = strtod(token, nullptr);
                break;
            case 14:
                f.dgpsStationId = static_cast<int>(strtol(token, nullptr, 10));
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
#endif  // GPGGA_H
