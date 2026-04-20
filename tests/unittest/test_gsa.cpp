#include "test_utils.h"
#include "nmea/gsa.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────
// 完整帧：GPGSA，3D 定位，8 颗卫星占据前 8 个 PRN slot，后 4 个为空
static const char GPGSA_FRAME[] = "$GPGSA,A,3,01,02,12,14,15,16,25,26,,,,, 1.8,1.0,1.5*1B\r\n";

// GNGSA：不同 Talker ID，2D 定位，4 颗卫星
static const char GNGSA_FRAME[] = "$GNGSA,A,2,05,08,10,13,,,,,,,,,2.5,1.5,2.0*22\r\n";

// 无定位（fixMode=1）：PRN slot 全空
static const char GPGSA_NOFIX_FRAME[] = "$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n";

// 部分空字段（PRN slot 与 DOP 字段混合缺省）
static const char GPGSA_PARTIAL_FRAME[] = "$GPGSA,M,3,01,,12,,15,,,,,,,, 1.2,,1.8*3C\r\n";

// 全空字段
static const char GPGSA_EMPTY_FRAME[] = "$GPGSA,,,,,,,,,,,,,,,,,,*42\r\n";

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（GPGSA）：验证 talkerId + 所有字段 + 原始回调前缀
bool testGsaSingleFrame() {
    Nmea::Gsa captured{};
    bool      rawOk = false, frameOk = false;

    Nmea::GsaParser parser(
        [&](const Nmea::Gsa &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 6) && (memcmp(data, "$GPGSA", 6) == 0);
        });

    auto v = toVec(GPGSA_FRAME);
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && strcmp(captured.talkerId, "GP") == 0
           && captured.selectionMode == 'A'
           && captured.fixMode == 3
           && captured.prnCount == 8
           && captured.prn[0] == 1
           && captured.prn[1] == 2
           && captured.prn[7] == 26
           && captured.prn[8] == 0  // empty slot
           && fEq(captured.pdop, 1.8)
           && fEq(captured.hdop, 1.0)
           && fEq(captured.vdop, 1.5);
}

// 多 Talker ID：GP / GN 各一帧，验证 talkerId 与 fixMode 正确提取
bool testGsaMultipleTalkerIds() {
    struct Expected {
        const char *frame;
        const char *id;
        int         fixMode;
        int         prnCount;
    };
    constexpr Expected cases[] = {
        {GPGSA_FRAME, "GP", 3, 8},
        {GNGSA_FRAME, "GN", 2, 4},
    };
    int             idx   = 0;
    bool            allOk = true;
    Nmea::GsaParser parser(
        [&](const Nmea::Gsa &f) {
            if(idx < 2) {
                allOk = allOk
                        && strcmp(f.talkerId, cases[idx].id) == 0
                        && f.fixMode == cases[idx].fixMode
                        && f.prnCount == cases[idx].prnCount;
                idx++;
            }
        },
        nullptr);
    for(auto &c : cases) {
        auto v = toVec(c.frame);
        parser.Parse(v.data(), v.size());
    }
    return allOk && idx == 2;
}

// fixMode=1（无定位）：所有 PRN slot 为空
bool testGsaNoFix() {
    Nmea::Gsa       captured{};
    Nmea::GsaParser parser(

        [&](const Nmea::Gsa &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPGSA_NOFIX_FRAME);
    parser.Parse(v.data(), v.size());

    return captured.fixMode == 1
           && captured.prnCount == 0
           && captured.prn[0] == 0;
}

// 部分空字段：PRN slot 与 DOP 字段混合缺省，验证索引不错位
bool testGsaPartialEmptyFields() {
    Nmea::Gsa       captured{};
    Nmea::GsaParser parser(
        [&](const Nmea::Gsa &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPGSA_PARTIAL_FRAME);
    parser.Parse(v.data(), v.size());

    // mode=M, fix=3, prn[0]=1, prn[1]=0(empty), prn[2]=12, prn[3]=0, prn[4]=15
    return captured.selectionMode == 'M'
           && captured.fixMode == 3
           && captured.prn[0] == 1
           && captured.prn[1] == 0  // empty slot
           && captured.prn[2] == 12
           && captured.prn[3] == 0  // empty slot
           && captured.prn[4] == 15
           && captured.prnCount == 3
           && fEq(captured.pdop, 1.2)
           && fEq(captured.hdop, 0.0)  // empty → 0
           && fEq(captured.vdop, 1.8);
}

// 全空字段：数值归零，字符字段为 '\0'，talkerId 仍正确
bool testGsaEmptyFields() {
    Nmea::Gsa       captured{};
    Nmea::GsaParser parser(
        [&](const Nmea::Gsa &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPGSA_EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return strcmp(captured.talkerId, "GP") == 0
           && captured.selectionMode == '\0'
           && captured.fixMode == 0
           && captured.prnCount == 0
           && fEq(captured.pdop, 0.0)
           && fEq(captured.hdop, 0.0)
           && fEq(captured.vdop, 0.0);
}

// 粘包：三帧直接拼接，期望 3 次回调
bool testGsaStickyPacket() {
    int             count = 0;
    Nmea::GsaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPGSA_FRAME);
    vecAppend(buf, toVec(GNGSA_FRAME));
    vecAppend(buf, toVec(GPGSA_NOFIX_FRAME));

    parser.Parse(buf.data(), buf.size());
    return count == 3;
}

// 分包（逐字节）
bool testGsaSplitPacketByte() {
    bool            rawOk = false;
    Nmea::GsaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });

    for(uint8_t b : toVec(GPGSA_FRAME)) {
        parser.ParseByteStream(b);
    }
    return rawOk;
}

// 分包（多片）：GPGSA + GNGSA 拼为一流，分三片送入
bool testGsaSplitPacketChunks() {
    int             count = 0;
    Nmea::GsaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPGSA_FRAME);
    vecAppend(buf, toVec(GNGSA_FRAME));

    uint32_t c1 = static_cast<uint32_t>(buf.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(buf.size() * 2 / 3);
    parser.Parse(buf.data(), c1);
    parser.Parse(buf.data() + c1, c2 - c1);
    parser.Parse(buf.data() + c2, static_cast<uint32_t>(buf.size()) - c2);
    return count == 2;
}

// 垃圾数据：假头 $GPGSB（最后字母不是 A）不会被误识别
bool testGsaWithGarbage() {
    int             count = 0;
    Nmea::GsaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });
    // '$','G','P','G','S',<非'A'> → 状态机回退
    const uint8_t        garbage[] = {0xFF, 0x01, '$', 'G', 'P', 'G', 'S', 0x01};
    std::vector<uint8_t> stream;
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GPGSA_FRAME));
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GNGSA_FRAME));

    parser.Parse(stream.data(), static_cast<uint32_t>(stream.size()));
    return count == 2;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_gsa ===\n\n";
    CHECK(testGsaSingleFrame());
    CHECK(testGsaMultipleTalkerIds());
    CHECK(testGsaNoFix());
    CHECK(testGsaPartialEmptyFields());
    CHECK(testGsaEmptyFields());
    CHECK(testGsaStickyPacket());
    CHECK(testGsaSplitPacketByte());
    CHECK(testGsaSplitPacketChunks());
    CHECK(testGsaWithGarbage());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
