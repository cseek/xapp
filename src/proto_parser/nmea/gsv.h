#ifndef GSV_H
#define GSV_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "nmea_checksum.h"

namespace Nmea {
// $GPGSV,numMsg,msgNum,numSat,[prn,elev,azim,snr,]*hh\r\n
// up to 4 satellite entries per message
struct GsvSatellite {
    int prn;        // satellite PRN / ID
    int elevation;  // elevation angle (degrees, 0-90)
    int azimuth;    // azimuth angle (degrees, 0-359)
    int snr;        // signal-to-noise ratio (dB-Hz, -1 = not tracking)
};

struct Gsv {
    char         talkerId[3];       // talker ID, e.g., "GP"
    int          totalMessages;     // total number of GSV sentences for this talker
    int          messageNumber;     // this sentence index (1-based)
    int          satellitesInView;  // total satellites in view
    int          satelliteCount;    // entries filled in this sentence (0-4)
    GsvSatellite satellites[4];     // satellite data for this sentence
};

class GsvParser {
public:
    using FrameCallback    = std::function<void(const Gsv &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    enum class State : uint8_t {
        DOLLAR,     // $
        TALKER1,    // first char of talker ID (any uppercase letter)
        TALKER2,    // second char of talker ID (any uppercase letter)
        SENTENCE1,  // G
        SENTENCE2,  // S
        SENTENCE3,  // V
        COMMA,      // ,
        DATA,       // data payload until \r
        LF          // \n
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[1024];  // $GPGSV,...*hh\r\n
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit GsvParser(
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
            Select(byte, 'S', State::SENTENCE3);
            break;

        case State::SENTENCE3:
            Select(byte, 'V', State::COMMA);
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
                    Gsv frame{};
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

    inline auto ParserFrame(Gsv &f) -> bool {
        f = Gsv{};
        char     buf[1024];
        uint32_t len = m_index + 1;  // include '\n'
        memcpy(buf, m_frame, len);
        buf[len] = '\0';

        if(!validateNmeaChecksum(buf)) {
            return false;
        }

        // Use pointer-based split to preserve empty fields (strtok collapses them).
        // fields: $XXGSV,totalMsg,msgNum,totalSat,
        //         [prn,elev,azim,snr,] ... *hh\r\n
        // satellite blocks start at field 4, 4 fields each
        char *p     = buf;
        int   field = 0;
        while(p != nullptr) {
            // find next comma, \r, or \n
            char *sep = p;
            while(*sep != '\0' && *sep != ',' && *sep != '\r' && *sep != '\n') {
                sep++;
            }
            char endChar = *sep;
            *sep         = '\0';  // terminate current field in-place
            char *token  = p;
            // advance p to next field (or nullptr if end)
            p = (endChar == ',') ? sep + 1 : nullptr;

            // strip trailing '*hh' checksum from current token
            char *star = strchr(token, '*');
            if(star != nullptr) {
                *star = '\0';
            }

            if(field == 0) {
                // $XXGSV — extract talker ID from chars [1..2]
                if(strlen(token) >= 3) {
                    f.talkerId[0] = token[1];
                    f.talkerId[1] = token[2];
                    f.talkerId[2] = '\0';
                }
            } else if(field == 1) {
                f.totalMessages = static_cast<int>(strtol(token, nullptr, 10));
            } else if(field == 2) {
                f.messageNumber = static_cast<int>(strtol(token, nullptr, 10));
            } else if(field == 3) {
                f.satellitesInView = static_cast<int>(strtol(token, nullptr, 10));
            } else {
                // satellite block: fields 4,5,6,7 → sat[0]; 8,9,10,11 → sat[1]; ...
                int offset = field - 4;
                int satIdx = offset / 4;
                int subIdx = offset % 4;
                if(satIdx < 4) {
                    switch(subIdx) {
                    case 0:  // PRN
                        f.satellites[satIdx].prn = static_cast<int>(strtol(token, nullptr, 10));
                        break;
                    case 1:  // elevation
                        f.satellites[satIdx].elevation = static_cast<int>(strtol(token, nullptr, 10));
                        break;
                    case 2:  // azimuth
                        f.satellites[satIdx].azimuth = static_cast<int>(strtol(token, nullptr, 10));
                        break;
                    case 3:  // SNR (may be empty when not tracking)
                        f.satellites[satIdx].snr = (token[0] != '\0')
                                                       ? static_cast<int>(strtol(token, nullptr, 10))
                                                       : -1;
                        if(satIdx + 1 > f.satelliteCount) {
                            f.satelliteCount = satIdx + 1;
                        }
                        break;
                    default:
                        break;
                    }
                }
            }
            field++;
        }
        return field >= 4;
    }
};
}  // namespace Nmea

#endif  // GSV_H
