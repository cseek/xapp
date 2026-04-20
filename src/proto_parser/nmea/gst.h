#ifndef GST_H
#define GST_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "nmea_checksum.h"

namespace Nmea {
// $XXGST,hhmmss.ss,rms,semiMajor,semiMinor,orient,latErr,lonErr,altErr*hh\r\n
// fields:
//  0  $XXGST           talker ID
//  1  hhmmss.ss        UTC time
//  2  x.x              RMS of the std-dev of the range inputs to the nav process
//  3  x.x              std-dev of semi-major axis of error ellipse (meters)
//  4  x.x              std-dev of semi-minor axis of error ellipse (meters)
//  5  x.x              orientation of semi-major axis, degrees from true north
//  6  x.x              std-dev of latitude error (meters)
//  7  x.x              std-dev of longitude error (meters)
//  8  x.x              std-dev of altitude error (meters)
struct Gst {
    char   talkerId[3];  // talker ID, e.g., "GP"
    double utcTime;      // UTC time, hhmmss.ss
    double rms;          // RMS of range-input std-dev
    double semiMajor;    // std-dev of semi-major axis (m)
    double semiMinor;    // std-dev of semi-minor axis (m)
    double orientation;  // orientation of semi-major axis (deg from N)
    double latErr;       // std-dev of latitude error (m)
    double lonErr;       // std-dev of longitude error (m)
    double altErr;       // std-dev of altitude error (m)
};

class GstParser {
public:
    using FrameCallback    = std::function<void(const Gst &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    enum class State : uint8_t {
        DOLLAR,     // $
        TALKER1,    // first uppercase letter of talker ID
        TALKER2,    // second uppercase letter of talker ID
        SENTENCE1,  // G
        SENTENCE2,  // S
        SENTENCE3,  // T
        COMMA,      // ,
        DATA,       // payload until \r
        LF          // \n
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[256];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit GstParser(
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
            Select(byte, 'G', State::SENTENCE2);
            break;
        case State::SENTENCE2:
            Select(byte, 'S', State::SENTENCE3);
            break;
        case State::SENTENCE3:
            Select(byte, 'T', State::COMMA);
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
                    Gst frame{};
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

    inline bool ParserFrame(Gst &f) {
        f = Gst{};
        char     buf[256];
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
            case 0:  // $XXGST — talker ID at [1..2]
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
                f.rms = strtod(token, nullptr);
                break;
            case 3:
                f.semiMajor = strtod(token, nullptr);
                break;
            case 4:
                f.semiMinor = strtod(token, nullptr);
                break;
            case 5:
                f.orientation = strtod(token, nullptr);
                break;
            case 6:
                f.latErr = strtod(token, nullptr);
                break;
            case 7:
                f.lonErr = strtod(token, nullptr);
                break;
            case 8:
                f.altErr = strtod(token, nullptr);
                break;
            default:
                break;
            }
            field++;
        }
        return field >= 9;
    }
};
}  // namespace Nmea

#endif  // GST_H
