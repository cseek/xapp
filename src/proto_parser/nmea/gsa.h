#ifndef GSA_H
#define GSA_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "nmea_checksum.h"

namespace Nmea {
// $XXGSA,mode1,mode2,prn01,...,prn12,PDOP,HDOP,VDOP*hh\r\n
// fields:
//  0      $XXGSA    talker ID
//  1      A/M       selection mode  (A=auto, M=manual)
//  2      1/2/3     fix mode        (1=none, 2=2D, 3=3D)
//  3-14   xx        up to 12 PRN slots (may be empty)
//  15     x.x       PDOP
//  16     x.x       HDOP
//  17     x.x       VDOP
struct Gsa {
    char   talkerId[3];    // talker ID, e.g., "GP"
    char   selectionMode;  // A=auto, M=manual, '\0' if absent
    int    fixMode;        // 1=no fix, 2=2D, 3=3D, 0 if absent
    int    prn[12];        // satellite PRNs in use (0 = empty slot)
    int    prnCount;       // number of non-zero PRN entries
    double pdop;
    double hdop;
    double vdop;
};

class GsaParser {
public:
    using FrameCallback    = std::function<void(const Gsa &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    enum class State : uint8_t {
        DOLLAR,     // $
        TALKER1,    // first uppercase letter of talker ID
        TALKER2,    // second uppercase letter of talker ID
        SENTENCE1,  // G
        SENTENCE2,  // S
        SENTENCE3,  // A
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
    explicit GsaParser(
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
                    Gsa frame{};
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

    inline bool ParserFrame(Gsa &f) {
        f = Gsa{};
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

            if(field == 0) {
                // $XXGSA — talker ID at [1..2]
                if(strlen(token) >= 3) {
                    f.talkerId[0] = token[1];
                    f.talkerId[1] = token[2];
                    f.talkerId[2] = '\0';
                }
            } else if(field == 1) {
                f.selectionMode = token[0];
            } else if(field == 2) {
                f.fixMode = token[0] != '\0'
                                ? static_cast<int>(strtol(token, nullptr, 10))
                                : 0;
            } else if(field >= 3 && field <= 14) {
                // PRN slots 0-11 (fields 3-14); empty slot → 0
                int slotIdx = field - 3;
                if(token[0] != '\0') {
                    int prn        = static_cast<int>(strtol(token, nullptr, 10));
                    f.prn[slotIdx] = prn;
                    if(prn != 0) {
                        f.prnCount++;
                    }
                }
            } else if(field == 15) {
                f.pdop = strtod(token, nullptr);
            } else if(field == 16) {
                f.hdop = strtod(token, nullptr);
            } else if(field == 17) {
                f.vdop = strtod(token, nullptr);
            }
            field++;
        }
        return field >= 18;
    }
};
}  // namespace Nmea
#endif  // GSA_H
