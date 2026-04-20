#include "test_utils.h"
#include "nmea/gst.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────
// 完整帧：GPGST，所有字段有值
static const char GPGST_FRAME[] = "$GPGST,092725.00,1.5,2.3,1.1,45.0,0.8,1.2,2.5*64\r\n";

// GNGST：不同 Talker ID
static const char GNGST_FRAME[] = "$GNGST,092725.00,0.9,1.8,0.7,30.0,0.5,0.6,1.0*74\r\n";

// 部分字段为空（RMS 和 orientation 缺省）
static const char GPGST_PARTIAL_FRAME[] = "$GPGST,092725.00,,2.3,1.1,,0.8,1.2,2.5*51\r\n";

// 全空字段
static const char GPGST_EMPTY_FRAME[] = "$GPGST,,,,,,,,*57\r\n";

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（GPGST）：验证 talkerId + 所有字段 + 原始回调前缀
bool testGstSingleFrame() {
    Nmea::Gst captured{};
    bool      rawOk = false, frameOk = false;

    Nmea::GstParser parser(
        [&](const Nmea::Gst &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 6) && (memcmp(data, "$GPGST", 6) == 0);
        });

    auto v = toVec(GPGST_FRAME);
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && strcmp(captured.talkerId, "GP") == 0
           && fEq(captured.utcTime, 92725.00)
           && fEq(captured.rms, 1.5)
           && fEq(captured.semiMajor, 2.3)
           && fEq(captured.semiMinor, 1.1)
           && fEq(captured.orientation, 45.0)
           && fEq(captured.latErr, 0.8)
           && fEq(captured.lonErr, 1.2)
           && fEq(captured.altErr, 2.5);
}

// 多 Talker ID：GP / GN 各一帧，验证 talkerId 正确提取
bool testGstMultipleTalkerIds() {
    struct Expected {
        const char *frame;
        const char *id;
        double      rms;
    };
    constexpr Expected cases[] = {
        {GPGST_FRAME, "GP", 1.5},
        {GNGST_FRAME, "GN", 0.9},
    };
    int             idx   = 0;
    bool            allOk = true;
    Nmea::GstParser parser(
        [&](const Nmea::Gst &f) {
            if(idx < 2) {
                allOk = allOk
                        && strcmp(f.talkerId, cases[idx].id) == 0
                        && fEq(f.rms, cases[idx].rms);
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

// 部分空字段：rms 和 orientation 为空，其余字段不错位
bool testGstPartialEmptyFields() {
    Nmea::Gst       captured{};
    Nmea::GstParser parser(
        [&](const Nmea::Gst &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPGST_PARTIAL_FRAME);
    parser.Parse(v.data(), v.size());

    return strcmp(captured.talkerId, "GP") == 0
           && fEq(captured.utcTime, 92725.00)
           && fEq(captured.rms, 0.0)  // 空 → 0
           && fEq(captured.semiMajor, 2.3)
           && fEq(captured.semiMinor, 1.1)
           && fEq(captured.orientation, 0.0)  // 空 → 0
           && fEq(captured.latErr, 0.8)
           && fEq(captured.lonErr, 1.2)
           && fEq(captured.altErr, 2.5);
}

// 全空字段：数值全部归零，talkerId 仍正确
bool testGstEmptyFields() {
    Nmea::Gst       captured{};
    Nmea::GstParser parser(
        [&](const Nmea::Gst &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPGST_EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return strcmp(captured.talkerId, "GP") == 0
           && fEq(captured.utcTime, 0.0)
           && fEq(captured.rms, 0.0)
           && fEq(captured.semiMajor, 0.0)
           && fEq(captured.semiMinor, 0.0)
           && fEq(captured.orientation, 0.0)
           && fEq(captured.latErr, 0.0)
           && fEq(captured.lonErr, 0.0)
           && fEq(captured.altErr, 0.0);
}

// 粘包：三帧直接拼接，期望 3 次回调
bool testGstStickyPacket() {
    int             count = 0;
    Nmea::GstParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPGST_FRAME);
    vecAppend(buf, toVec(GNGST_FRAME));
    vecAppend(buf, toVec(GPGST_PARTIAL_FRAME));

    parser.Parse(buf.data(), buf.size());
    return count == 3;
}

// 分包（逐字节）
bool testGstSplitPacketByte() {
    bool            rawOk = false;
    Nmea::GstParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });

    for(uint8_t b : toVec(GPGST_FRAME)) {
        parser.ParseByteStream(b);
    }
    return rawOk;
}

// 分包（多片）：GPGST + GNGST 拼为一流，分三片送入
bool testGstSplitPacketChunks() {
    int             count = 0;
    Nmea::GstParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPGST_FRAME);
    vecAppend(buf, toVec(GNGST_FRAME));

    uint32_t c1 = static_cast<uint32_t>(buf.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(buf.size() * 2 / 3);
    parser.Parse(buf.data(), c1);
    parser.Parse(buf.data() + c1, c2 - c1);
    parser.Parse(buf.data() + c2, static_cast<uint32_t>(buf.size()) - c2);
    return count == 2;
}

// 垃圾数据：假头 $GPGSX（最后字母不是 T）不会被误识别
bool testGstWithGarbage() {
    int             count = 0;
    Nmea::GstParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    // '$','G','P','G','S',<非'T'> → 状态机回退
    const uint8_t        garbage[] = {0xFF, 0x01, '$', 'G', 'P', 'G', 'S', 0x01};
    std::vector<uint8_t> stream;
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GPGST_FRAME));
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GNGST_FRAME));

    parser.Parse(stream.data(), static_cast<uint32_t>(stream.size()));
    return count == 2;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_gst ===\n\n";
    CHECK(testGstSingleFrame());
    CHECK(testGstMultipleTalkerIds());
    CHECK(testGstPartialEmptyFields());
    CHECK(testGstEmptyFields());
    CHECK(testGstStickyPacket());
    CHECK(testGstSplitPacketByte());
    CHECK(testGstSplitPacketChunks());
    CHECK(testGstWithGarbage());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
