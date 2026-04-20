#ifndef RAWIMUA_H
#define RAWIMUA_H

#include <functional>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "novatel_ascii_crc.h"

namespace Novatel {
// #RAWIMUA,<port>,<seq>,<idle%>,<time_status>,<week>,<sec>,<rx_status>,<hdr_len>,<msg_len>;
// <week>,<sec>,<imu_status>,<z_accel>,<neg_y_accel>,<x_accel>,<z_gyro>,<neg_y_gyro>,<x_gyro>*<CRC32>
struct RawImuA {
    uint32_t gpsWeek;     // GPS week number (from body)
    double   gpsSeconds;  // GPS seconds of week (from body)
    uint32_t imuStatus;   // IMU status word
    int32_t  zAccel;      // Z-axis accelerometer (raw count)
    int32_t  negYAccel;   // -Y-axis accelerometer (raw count)
    int32_t  xAccel;      // X-axis accelerometer (raw count)
    int32_t  zGyro;       // Z-axis gyro (raw count)
    int32_t  negYGyro;    // -Y-axis gyro (raw count)
    int32_t  xGyro;       // X-axis gyro (raw count)
};

class RawImuAParser {
public:
    using FrameCallback    = std::function<void(const RawImuA &frame)>;
    using RawFrameCallback = std::function<void(uint8_t *data, uint32_t len)>;

private:
    enum class State : uint8_t {
        SHARP,    // #
        HEADER1,  // R
        HEADER2,  // A
        HEADER3,  // W
        HEADER4,  // I
        HEADER5,  // M
        HEADER6,  // U
        HEADER7,  // A
        COMMA,    // ,
        CR,       // \r
        LF        // \n
    };
    State            m_state;
    uint32_t         m_index;
    uint8_t          m_frame[1024];  // #RAWIMUA,XXX,XXX,....*AABBCCDD\r\n
    FrameCallback    m_frameCallback;
    RawFrameCallback m_rawFrameCallback;

public:
    explicit RawImuAParser(
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
        for(auto i = 0; i < size; i++) {
            ParseByteStream(data[i]);
        }
    }

    void ParseByteStream(uint8_t byte) {
        switch(m_state) {
        case State::SHARP:
            Select(byte, '#', State::HEADER1);
            break;

        case State::HEADER1:
            Select(byte, 'R', State::HEADER2);
            break;

        case State::HEADER2:
            Select(byte, 'A', State::HEADER3);
            break;

        case State::HEADER3:
            Select(byte, 'W', State::HEADER4);
            break;

        case State::HEADER4:
            Select(byte, 'I', State::HEADER5);
            break;

        case State::HEADER5:
            Select(byte, 'M', State::HEADER6);
            break;

        case State::HEADER6:
            Select(byte, 'U', State::HEADER7);
            break;

        case State::HEADER7:
            Select(byte, 'A', State::COMMA);
            break;

        case State::COMMA:
            Select(byte, ',', State::CR);
            break;

        case State::CR:
            m_frame[m_index++] = byte;
            if(m_index > sizeof(m_frame) - 2) {
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
                    RawImuA frame{};
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

    inline auto ParserFrame(RawImuA &f) -> bool {
        f = RawImuA{};
        char     buf[1024];
        uint32_t len = m_index + 1;  // include '\n'
        memcpy(buf, m_frame, len);
        buf[len] = '\0';

        if(!validateAsciiFrameCrc32(buf, '#')) {
            return false;
        }

        // Use pointer-based split to preserve empty fields.
        // delimiters: comma, semicolon (header/body separator), asterisk (before CRC), CR, LF
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
            case 0:  // #RAWIMUA — skip
                break;
            case 1:  // port — skip
                break;
            case 2:  // sequence number — skip
                break;
            case 3:  // idle % — skip
                break;
            case 4:  // time status — skip
                break;
            case 5:  // GPS week (header) — skip, use body value
                break;
            case 6:  // GPS seconds (header) — skip, use body value
                break;
            case 7:  // receiver status — skip
                break;
            case 8:  // header length — skip
                break;
            case 9:  // message length — skip
                break;
            case 10:  // GPS week (body)
                f.gpsWeek = static_cast<uint32_t>(strtoul(token, nullptr, 10));
                break;
            case 11:  // GPS seconds (body)
                f.gpsSeconds = strtod(token, nullptr);
                break;
            case 12:  // IMU status (hex)
                f.imuStatus = static_cast<uint32_t>(strtoul(token, nullptr, 16));
                break;
            case 13:  // Z accelerometer
                f.zAccel = static_cast<int32_t>(strtol(token, nullptr, 10));
                break;
            case 14:  // -Y accelerometer
                f.negYAccel = static_cast<int32_t>(strtol(token, nullptr, 10));
                break;
            case 15:  // X accelerometer
                f.xAccel = static_cast<int32_t>(strtol(token, nullptr, 10));
                break;
            case 16:  // Z gyro
                f.zGyro = static_cast<int32_t>(strtol(token, nullptr, 10));
                break;
            case 17:  // -Y gyro
                f.negYGyro = static_cast<int32_t>(strtol(token, nullptr, 10));
                break;
            case 18:  // X gyro
                f.xGyro = static_cast<int32_t>(strtol(token, nullptr, 10));
                break;
            default:
                break;
            }
            field++;
        }
        return field >= 19;
    }
};
}  // namespace Novatel

#endif  // RAWIMUA_H