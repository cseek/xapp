#include "test_utils.h"
#include "novatel/dualantennaheadinga.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────

// 官方文档示例帧（NARROW_INT，heading/pitch 均有效）
static const char DAH_FRAME1[] = "#DUALANTENNAHEADINGA,UNKNOWN,0,66.5,FINESTEERING,1949,575614.000,02000000,d426,32768;"
                                 "SOL_COMPUTED,NARROW_INT,-1.000000000,255.538528442,0.006041416,0.0,"
                                 "0.043859947,0.052394450,\"J56X\",24,18,18,17,04,01,00,33*1f082ec5\r\n";

// 第二帧：heading/pitch 不同值，差分龄期与 solSource 非零
static const char DAH_FRAME2[] = "#DUALANTENNAHEADINGA,COM1,1,72.0,FINESTEERING,2344,405000.000,02000020,d426,32768;"
                                 "SOL_COMPUTED,NARROW_FLOAT,1.230000000,90.123456789,-3.456789000,0.0,"
                                 "0.120000005,0.080000001,\"BASE\",20,16,15,12,04,02,01,55*62535e26\r\n";

// 解算失败帧
static const char DAH_FRAME3[] = "#DUALANTENNAHEADINGA,COM1,2,80.0,COARSESTEERING,2344,405001.000,02000020,d426,32768;"
                                 "INSUFFICIENT_OBS,NONE,0.0,0.0,0.0,0.0,0.0,0.0,\"\",0,0,0,0,00,00,00,00*ab7f03b7\r\n";

// 空字段帧（heading/pitch 空）
static const char DAH_EMPTY_FRAME[] = "#DUALANTENNAHEADINGA,COM1,3,70.0,FINESTEERING,2344,405002.000,02000020,d426,32768;"
                                      "SOL_COMPUTED,NARROW_INT,-1.0,,0.0,0.0,,0.05,\"A1\",10,8,8,6,04,01,00,33*c93d46b0\r\n";

// ── 辅助 ─────────────────────────────────────────────────────────────────────

static std::vector<uint8_t> toVecDAH(const char *s) {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(s);
    return {p, p + strlen(s)};
}

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（FRAME1）：所有字段校验，并检查原始回调前缀
bool testDAHSingleFrame() {
    Novatel::DualAntennaHeadingA captured{};
    bool                         rawOk = false, frameOk = false;

    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 20) && (memcmp(data, "#DUALANTENNAHEADINGA", 20) == 0);
        });

    auto v = toVecDAH(DAH_FRAME1);
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && strcmp(captured.posType, "NARROW_INT") == 0
           && fEq(captured.length, -1.0f, 1e-4f)
           && fEq(captured.heading, 255.538528f, 1e-3f)
           && fEq(captured.pitch, 0.006041f, 1e-5f)
           && fEq(captured.hdgStdDev, 0.043859f, 1e-4f)
           && fEq(captured.ptchStdDev, 0.052394f, 1e-4f)
           && strcmp(captured.stnId, "J56X") == 0
           && captured.numSvs == 24
           && captured.numSolnSvs == 18
           && captured.numObs == 18
           && captured.numMulti == 17
           && captured.solSource == 0x04
           && captured.extSolStat == 0x01
           && captured.galBdsSigMask == 0x00
           && captured.gpsGloSigMask == 0x33;
}

// 第二帧：NARROW_FLOAT，heading/pitch 正负值
bool testDAHFrame2() {
    Novatel::DualAntennaHeadingA captured{};
    bool                         frameOk = false;

    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecDAH(DAH_FRAME2);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.posType, "NARROW_FLOAT") == 0
           && fEq(captured.heading, 90.123456f, 1e-3f)
           && fEq(captured.pitch, -3.456789f, 1e-4f)
           && strcmp(captured.stnId, "BASE") == 0
           && captured.numSvs == 20
           && captured.numSolnSvs == 16
           && captured.gpsGloSigMask == 0x55;
}

// 解算失败帧：frameCallback 触发，字段为零/NONE
bool testDAHInsufficientObs() {
    Novatel::DualAntennaHeadingA captured{};
    bool                         frameOk = false;

    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecDAH(DAH_FRAME3);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.solStatus, "INSUFFICIENT_OBS") == 0
           && strcmp(captured.posType, "NONE") == 0
           && fEq(captured.heading, 0.0f, 1e-6f)
           && fEq(captured.pitch, 0.0f, 1e-6f)
           && captured.numSvs == 0;
}

// 逐字节解析：验证状态机流式能力
bool testDAHByteByByte() {
    Novatel::DualAntennaHeadingA captured{};
    bool                         frameOk = false;

    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    for(size_t i = 0; i < strlen(DAH_FRAME1); i++) {
        parser.ParseByteStream(static_cast<uint8_t>(DAH_FRAME1[i]));
    }

    return frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && fEq(captured.heading, 255.538528f, 1e-3f);
}

// 连续两帧：各自独立触发
bool testDAHMultipleFrames() {
    int                          callCount = 0;
    Novatel::DualAntennaHeadingA frames[2]{};

    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &f) {
            if(callCount < 2) {
                frames[callCount] = f;
            }
            callCount++;
        },
        nullptr);

    auto v1 = toVecDAH(DAH_FRAME1);
    auto v2 = toVecDAH(DAH_FRAME2);
    parser.Parse(v1.data(), v1.size());
    parser.Parse(v2.data(), v2.size());

    return callCount == 2
           && strcmp(frames[0].posType, "NARROW_INT") == 0
           && strcmp(frames[1].posType, "NARROW_FLOAT") == 0;
}

// 垃圾数据 + 有效帧：状态机正确过滤
bool testDAHWithGarbage() {
    Novatel::DualAntennaHeadingA captured{};
    bool                         frameOk = false;

    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    const char     garbage[] = "GARBAGE#DUAL##NOISELINE\r\n";
    const uint8_t *gj        = reinterpret_cast<const uint8_t *>(garbage);
    parser.Parse(gj, strlen(garbage));

    auto v = toVecDAH(DAH_FRAME1);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && fEq(captured.heading, 255.538528f, 1e-3f);
}

// 空字段：heading/hdgStdDev 为空应解析为 0
bool testDAHEmptyFields() {
    Novatel::DualAntennaHeadingA captured{};
    bool                         frameOk = false;

    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecDAH(DAH_EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && fEq(captured.length, -1.0f, 1e-4f)
           && fEq(captured.heading, 0.0f, 1e-6f)    // empty → 0
           && fEq(captured.hdgStdDev, 0.0f, 1e-6f)  // empty → 0
           && fEq(captured.ptchStdDev, 0.05f, 1e-4f)
           && strcmp(captured.stnId, "A1") == 0
           && captured.numSvs == 10;
}

// 仅原始回调，无 frame 回调：不崩溃
bool testDAHRawCallbackOnly() {
    bool                               rawOk = false;
    Novatel::DualAntennaHeadingAParser parser(
        nullptr,
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 0);
        });

    auto v = toVecDAH(DAH_FRAME1);
    parser.Parse(v.data(), v.size());
    return rawOk;
}

// 粘包：两帧连续送入同一字节流
bool testDAHStickyPacket() {
    int                                callCount = 0;
    Novatel::DualAntennaHeadingAParser parser(
        [&](const Novatel::DualAntennaHeadingA &) {
            callCount++;
        },
        nullptr);

    std::vector<uint8_t> combined;
    for(char c : std::string(DAH_FRAME1)) {
        combined.push_back(c);
    }
    for(char c : std::string(DAH_FRAME2)) {
        combined.push_back(c);
    }
    parser.Parse(combined.data(), combined.size());

    return callCount == 2;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "\n=== test_dualantennaheadinga ===\n\n";
    CHECK(testDAHSingleFrame());
    CHECK(testDAHFrame2());
    CHECK(testDAHInsufficientObs());
    CHECK(testDAHByteByByte());
    CHECK(testDAHMultipleFrames());
    CHECK(testDAHWithGarbage());
    CHECK(testDAHEmptyFields());
    CHECK(testDAHRawCallbackOnly());
    CHECK(testDAHStickyPacket());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
