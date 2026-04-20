#ifndef BESTGNSSPOSA_H
#define BESTGNSSPOSA_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "novatel_ascii_crc.h"

namespace Novatel {
// #BESTGNSSPOSA,<port>,<seq>,<idle%>,<time_status>,<week>,<sec>,<rx_status>,<hdr_len>,<msg_len>;
// <sol_status>,<pos_type>,<lat>,<lon>,<hgt>,<undulation>,<datum_id>,
// <lat_std>,<lon_std>,<hgt_std>,<stn_id>,<diff_age>,<sol_age>,
// <num_svs>,<num_sol_svs>,<num_gg_l1>,<num_sol_multi_svs>,<reserved>,
// <ext_sol_stat>,<galileo_beidou_sig_mask>,<gps_glonass_sig_mask>*<CRC32>
struct BestGnssPosA {
    char    solStatus[32];   // solution status string
    char    posType[32];     // position type string
    double  lat;             // latitude (degrees, positive=N)
    double  lon;             // longitude (degrees, positive=E)
    double  hgt;             // height above mean sea level (m)
    float   undulation;      // undulation (m)
    char    datumId[16];     // datum ID string
    float   latStd;          // latitude std dev (m)
    float   lonStd;          // longitude std dev (m)
    float   hgtStd;          // height std dev (m)
    char    stnId[8];        // base station ID
    float   diffAge;         // differential age (s)
    float   solAge;          // solution age (s)
    uint8_t numSvs;          // number of satellites tracked
    uint8_t numSolSvs;       // number of SVs used in solution
    uint8_t numGgL1;         // number of GPS+GLONASS L1 SVs
    uint8_t numSolMultiSvs;  // number of multi-freq SVs in solution
    uint8_t extSolStat;      // extended solution status
    uint8_t galBdsSigMask;   // Galileo/BeiDou signal used mask
    uint8_t gpsGloSigMask;   // GPS/GLONASS signal used mask
};

class BestGnssPosAParser {
public:
    using FrameCallback    = std::function<void(const BestGnssPosA &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    enum class State : uint8_t {
        SHARP,  // #
        H1,     // B
        H2,     // E
        H3,     // S
        H4,     // T
        H5,     // G
        H6,     // N
        H7,     // S
        H8,     // S
        H9,     // P
        H10,    // O
        H11,    // S
        H12,    // A
        CR,     // data accumulation until \r
        LF      // \n
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[1024];
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit BestGnssPosAParser(
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
            Select(byte, 'B', State::H2);
            break;
        case State::H2:
            Select(byte, 'E', State::H3);
            break;
        case State::H3:
            Select(byte, 'S', State::H4);
            break;
        case State::H4:
            Select(byte, 'T', State::H5);
            break;
        case State::H5:
            Select(byte, 'G', State::H6);
            break;
        case State::H6:
            Select(byte, 'N', State::H7);
            break;
        case State::H7:
            Select(byte, 'S', State::H8);
            break;
        case State::H8:
            Select(byte, 'S', State::H9);
            break;
        case State::H9:
            Select(byte, 'P', State::H10);
            break;
        case State::H10:
            Select(byte, 'O', State::H11);
            break;
        case State::H11:
            Select(byte, 'S', State::H12);
            break;
        case State::H12:
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
                    BestGnssPosA frame{};
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

    inline auto ParserFrame(BestGnssPosA &f) -> bool {
        f = BestGnssPosA{};
        char     buf[1024];
        uint32_t len = m_index + 1;  // include '\n'
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
            // ── header fields 0-9 ──────────────────────────────────────
            case 0:
                break;  // #BESTGNSSPOSA
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
            // ── body fields 10-30 ──────────────────────────────────────
            case 10:  // sol_status
                strncpy(f.solStatus, token, sizeof(f.solStatus) - 1);
                break;
            case 11:  // pos_type
                strncpy(f.posType, token, sizeof(f.posType) - 1);
                break;
            case 12:  // lat
                f.lat = strtod(token, nullptr);
                break;
            case 13:  // lon
                f.lon = strtod(token, nullptr);
                break;
            case 14:  // hgt
                f.hgt = strtod(token, nullptr);
                break;
            case 15:  // undulation
                f.undulation = strtof(token, nullptr);
                break;
            case 16:  // datum_id
                strncpy(f.datumId, token, sizeof(f.datumId) - 1);
                break;
            case 17:  // lat_std
                f.latStd = strtof(token, nullptr);
                break;
            case 18:  // lon_std
                f.lonStd = strtof(token, nullptr);
                break;
            case 19:  // hgt_std
                f.hgtStd = strtof(token, nullptr);
                break;
            case 20:  // stn_id
                strncpy(f.stnId, token, sizeof(f.stnId) - 1);
                break;
            case 21:  // diff_age
                f.diffAge = strtof(token, nullptr);
                break;
            case 22:  // sol_age
                f.solAge = strtof(token, nullptr);
                break;
            case 23:  // num_svs
                f.numSvs = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 24:  // num_sol_svs
                f.numSolSvs = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 25:  // num_gg_l1
                f.numGgL1 = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 26:  // num_sol_multi_svs
                f.numSolMultiSvs = static_cast<uint8_t>(strtoul(token, nullptr, 10));
                break;
            case 27:  // reserved — skip
                break;
            case 28:  // ext_sol_stat (hex)
                f.extSolStat = static_cast<uint8_t>(strtoul(token, nullptr, 16));
                break;
            case 29:  // galileo_beidou_sig_mask (hex)
                f.galBdsSigMask = static_cast<uint8_t>(strtoul(token, nullptr, 16));
                break;
            case 30:  // gps_glonass_sig_mask (hex)
                f.gpsGloSigMask = static_cast<uint8_t>(strtoul(token, nullptr, 16));
                break;
            default:
                break;
            }
            field++;
        }
        return field >= 31;
    }
};
}  // namespace Novatel

#endif  // BESTGNSSPOSA_H
