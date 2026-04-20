#ifndef DUALANTENNAHEADINGA_H
#define DUALANTENNAHEADINGA_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "novatel_ascii_crc.h"

namespace Novatel {
// #DUALANTENNAHEADINGA,<port>,<seq>,<idle%>,<time_status>,<week>,<sec>,<rx_status>,<hdr_len>,<msg_len>;
// <sol_status>,<pos_type>,<length>,<heading>,<pitch>,<reserved>,
// <hdg_std_dev>,<ptch_std_dev>,<stn_id>,
// <num_svs>,<num_soln_svs>,<num_obs>,<num_multi>,
// <sol_source>,<ext_sol_stat>,<gal_bds_sig_mask>,<gps_glo_sig_mask>*<CRC32>
struct DualAntennaHeadingA {
    char    solStatus[32];  // solution status string
    char    posType[32];    // position type string
    float   length;         // baseline length (m); -1 for ALIGN heading models
    float   heading;        // heading (0~359.999 deg)
    float   pitch;          // pitch (±90 deg)
    float   hdgStdDev;      // heading std dev (deg)
    float   ptchStdDev;     // pitch std dev (deg)
    char    stnId[8];       // station ID string
    uint8_t numSvs;         // number of satellites tracked
    uint8_t numSolnSvs;     // number of SVs used in solution
    uint8_t numObs;         // number of SVs above elevation mask
    uint8_t numMulti;       // number of multi-freq SVs in solution
    uint8_t solSource;      // solution source (hex)
    uint8_t extSolStat;     // extended solution status (hex)
    uint8_t galBdsSigMask;  // Galileo/BeiDou signal used mask (hex)
    uint8_t gpsGloSigMask;  // GPS/GLONASS signal used mask (hex)
};

class DualAntennaHeadingAParser {
public:
    using FrameCallback    = std::function<void(const DualAntennaHeadingA &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    // match: #DUALANTENNAHEADINGA  (19 chars after #)
    enum class State : uint8_t {
        SHARP,
        H1,   // D
        H2,   // U
        H3,   // A
        H4,   // L
        H5,   // A
        H6,   // N
        H7,   // T
        H8,   // E
        H9,   // N
        H10,  // N
        H11,  // A
        H12,  // H
        H13,  // E
        H14,  // A
        H15,  // D
        H16,  // I
        H17,  // N
        H18,  // G
        H19,  // A
        CR,
        LF
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[512];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit DualAntennaHeadingAParser(
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
            Select(byte, 'D', State::H2);
            break;
        case State::H2:
            Select(byte, 'U', State::H3);
            break;
        case State::H3:
            Select(byte, 'A', State::H4);
            break;
        case State::H4:
            Select(byte, 'L', State::H5);
            break;
        case State::H5:
            Select(byte, 'A', State::H6);
            break;
        case State::H6:
            Select(byte, 'N', State::H7);
            break;
        case State::H7:
            Select(byte, 'T', State::H8);
            break;
        case State::H8:
            Select(byte, 'E', State::H9);
            break;
        case State::H9:
            Select(byte, 'N', State::H10);
            break;
        case State::H10:
            Select(byte, 'N', State::H11);
            break;
        case State::H11:
            Select(byte, 'A', State::H12);
            break;
        case State::H12:
            Select(byte, 'H', State::H13);
            break;
        case State::H13:
            Select(byte, 'E', State::H14);
            break;
        case State::H14:
            Select(byte, 'A', State::H15);
            break;
        case State::H15:
            Select(byte, 'D', State::H16);
            break;
        case State::H16:
            Select(byte, 'I', State::H17);
            break;
        case State::H17:
            Select(byte, 'N', State::H18);
            break;
        case State::H18:
            Select(byte, 'G', State::H19);
            break;
        case State::H19:
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
                    DualAntennaHeadingA frame{};
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

    inline auto ParserFrame(DualAntennaHeadingA &f) -> bool {
        f = DualAntennaHeadingA{};
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
            // strip surrounding quotes if present (stn_id field)
            if(token[0] == '"') {
                token++;
                char *q = token;
                while(*q && *q != '"') {
                    q++;
                }
                *q = '\0';
            }
            p = (endChar == ',' || endChar == ';' || endChar == '*') ? sep + 1 : nullptr;

            switch(field) {
            // ── header 0-9 ──────────────────────────────────────────────
            case 0:
                break;  // #DUALANTENNAHEADINGA
            case 1:
                break;  // port
            case 2:
                break;  // sequence
            case 3:
                break;  // idle%
            case 4:
                break;  // time_status
            case 5:
                break;  // GPS week
            case 6:
                break;  // GPS seconds
            case 7:
                break;  // rx_status
            case 8:
                break;  // hdr_len
            case 9:
                break;  // msg_len
            // ── body 10-26 ──────────────────────────────────────────────
            case 10:
                strncpy(f.solStatus, token, sizeof(f.solStatus) - 1);
                break;
            case 11:
                strncpy(f.posType, token, sizeof(f.posType) - 1);
                break;
            case 12:
                f.length = strtof(token, nullptr);
                break;
            case 13:
                f.heading = strtof(token, nullptr);
                break;
            case 14:
                f.pitch = strtof(token, nullptr);
                break;
            case 15:
                break;  // reserved
            case 16:
                f.hdgStdDev = strtof(token, nullptr);
                break;
            case 17:
                f.ptchStdDev = strtof(token, nullptr);
                break;
            case 18:
                strncpy(f.stnId, token, sizeof(f.stnId) - 1);
                break;
            case 19:
                f.numSvs = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 20:
                f.numSolnSvs = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 21:
                f.numObs = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 22:
                f.numMulti = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 23:
                f.solSource = static_cast<uint8_t>(strtoul(token, nullptr, 16));
                break;
            case 24:
                f.extSolStat = static_cast<uint8_t>(strtoul(token, nullptr, 16));
                break;
            case 25:
                f.galBdsSigMask = static_cast<uint8_t>(strtoul(token, nullptr, 16));
                break;
            case 26:
                f.gpsGloSigMask = static_cast<uint8_t>(strtoul(token, nullptr, 16));
                break;
            default:
                break;
            }
            field++;
        }
        return field >= 27;
    }
};
}  // namespace Novatel

#endif  // DUALANTENNAHEADINGA_H
