#include "test_utils.h"
#include "novatel/bestgnssposa.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────

// 标准 SOL_COMPUTED / SINGLE 定位帧
static const char BESTGNSSPOSA_FRAME1[] = "#BESTGNSSPOSA,COM1,0,78.5,FINESTEERING,2344,405000.000,02000020,b1f4,32;"
                                          "SOL_COMPUTED,SINGLE,31.22819476,121.48917843,20.1230,-12.3400,WGS84,"
                                          "0.9100,0.7800,1.5600,\"\",0.000,1.020,14,12,10,8,0,06,00,33*4cc4caef\r\n";

// PSRDIFF 差分定位帧（diff_age 非零）
static const char BESTGNSSPOSA_FRAME2[] = "#BESTGNSSPOSA,COM1,1,75.0,FINESTEERING,2344,405001.000,02000020,b1f4,32;"
                                          "SOL_COMPUTED,PSRDIFF,39.90419857,116.40739628,45.6780,8.1200,WGS84,"
                                          "0.4500,0.3800,0.9900,\"BASE\",3.500,2.100,20,18,16,14,0,0E,01,55*51ce24fd\r\n";

// INSUFFICIENT_OBS — 解算失败帧（parserFrame 仍应成功，字段数满足）
static const char BESTGNSSPOSA_FRAME3[] = "#BESTGNSSPOSA,COM1,2,90.0,COARSESTEERING,2344,405002.000,02000020,b1f4,32;"
                                          "INSUFFICIENT_OBS,NONE,0.00000000,0.00000000,0.0000,0.0000,WGS84,"
                                          "0.0000,0.0000,0.0000,\"\",0.000,0.000,0,0,0,0,0,00,00,00*32f40bfa\r\n";

// 空字段帧（body 部分字段为空，测试 strtod/strtof 对空串返回 0）
static const char BESTGNSSPOSA_EMPTY_FRAME[] = "#BESTGNSSPOSA,COM1,3,80.0,FINESTEERING,2344,405003.000,02000020,b1f4,32;"
                                               "SOL_COMPUTED,SINGLE,,121.00000000,10.0000,0.0000,WGS84,"
                                               ",,1.0000,\"\",0.000,0.000,8,8,8,6,0,06,00,33*d371f875\r\n";

// ── 辅助 ──────────────────────────────────────────────────────────────────────

// 把 const char* 帧转换为字节向量
static std::vector<uint8_t> toVecBGP(const char *s) {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(s);
    return {p, p + strlen(s)};
}

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（FRAME1）：原始回调前缀校验 & 所有结构体字段
bool testBGPSingleFrame() {
    Novatel::BestGnssPosA       captured{};
    bool                        rawOk = false, frameOk = false;
    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 14) && (memcmp(data, "#BESTGNSSPOSA", 13) == 0);
        });

    auto v = toVecBGP(BESTGNSSPOSA_FRAME1);
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && strcmp(captured.posType, "SINGLE") == 0
           && fEq(captured.lat, 31.22819476, 1e-6)
           && fEq(captured.lon, 121.48917843, 1e-6)
           && fEq(captured.hgt, 20.1230, 1e-3)
           && fEq(captured.undulation, -12.3400, 1e-3)
           && strcmp(captured.datumId, "WGS84") == 0
           && fEq(captured.latStd, 0.9100f, 1e-3)
           && fEq(captured.lonStd, 0.7800f, 1e-3)
           && fEq(captured.hgtStd, 1.5600f, 1e-3)
           && fEq(captured.diffAge, 0.000f, 1e-3)
           && fEq(captured.solAge, 1.020f, 1e-3)
           && captured.numSvs == 14
           && captured.numSolSvs == 12
           && captured.numGgL1 == 10
           && captured.numSolMultiSvs == 8
           && captured.extSolStat == 0x06
           && captured.galBdsSigMask == 0x00
           && captured.gpsGloSigMask == 0x33;
}

// 差分帧（FRAME2）：差分相关字段校验
bool testBGPDiffFrame() {
    Novatel::BestGnssPosA captured{};
    bool                  frameOk = false;

    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecBGP(BESTGNSSPOSA_FRAME2);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && strcmp(captured.posType, "PSRDIFF") == 0
           && fEq(captured.lat, 39.90419857, 1e-6)
           && fEq(captured.lon, 116.40739628, 1e-6)
           && fEq(captured.diffAge, 3.500f, 1e-3)
           && fEq(captured.solAge, 2.100f, 1e-3)
           && captured.numSvs == 20
           && captured.numSolSvs == 18
           && captured.extSolStat == 0x0E
           && captured.galBdsSigMask == 0x01
           && captured.gpsGloSigMask == 0x55;
}

// 解算失败帧（FRAME3）：字段数满足阈值，frameCallback 应触发
bool testBGPInsufficientObs() {
    bool                  frameOk = false;
    Novatel::BestGnssPosA captured{};

    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecBGP(BESTGNSSPOSA_FRAME3);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.solStatus, "INSUFFICIENT_OBS") == 0
           && strcmp(captured.posType, "NONE") == 0
           && fEq(captured.lat, 0.0, 1e-9)
           && fEq(captured.lon, 0.0, 1e-9)
           && captured.numSvs == 0;
}

// 分块送入：逐字节解析（验证状态机流式能力）
bool testBGPByteByByte() {
    Novatel::BestGnssPosA captured{};
    bool                  frameOk = false;

    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    for(char c : std::vector<uint8_t>(
            reinterpret_cast<const uint8_t *>(BESTGNSSPOSA_FRAME1),
            reinterpret_cast<const uint8_t *>(BESTGNSSPOSA_FRAME1) + strlen(BESTGNSSPOSA_FRAME1))) {
        parser.ParseByteStream(static_cast<uint8_t>(c));
    }

    return frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && fEq(captured.lat, 31.22819476, 1e-6);
}

// 乱序垃圾 + 有效帧：状态机应正确过滤
bool testBGPWithGarbage() {
    Novatel::BestGnssPosA captured{};
    bool                  frameOk = false;

    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    const char     garbage[] = "JUNK_DATA$$NOISE#BAD_FRAME\r\n";
    const uint8_t *gj        = reinterpret_cast<const uint8_t *>(garbage);
    parser.Parse(gj, strlen(garbage));

    auto v = toVecBGP(BESTGNSSPOSA_FRAME1);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && fEq(captured.lat, 31.22819476, 1e-6);
}

// 连续两帧：各自触发独立回调
bool testBGPMultipleFrames() {
    int                   callCount = 0;
    Novatel::BestGnssPosA frames[2]{};

    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &f) {
            if(callCount < 2) {
                frames[callCount] = f;
            }
            callCount++;
        },
        nullptr);

    auto v1 = toVecBGP(BESTGNSSPOSA_FRAME1);
    auto v2 = toVecBGP(BESTGNSSPOSA_FRAME2);
    parser.Parse(v1.data(), v1.size());
    parser.Parse(v2.data(), v2.size());

    return callCount == 2
           && strcmp(frames[0].posType, "SINGLE") == 0
           && strcmp(frames[1].posType, "PSRDIFF") == 0;
}

// 空字段帧：空字段 lat/latStd/lonStd 应解析为 0
bool testBGPEmptyFields() {
    Novatel::BestGnssPosA captured{};
    bool                  frameOk = false;

    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecBGP(BESTGNSSPOSA_EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.solStatus, "SOL_COMPUTED") == 0
           && fEq(captured.lat, 0.0, 1e-9)  // empty → 0
           && fEq(captured.lon, 121.0, 1e-4)
           && fEq(captured.latStd, 0.0f, 1e-6)  // empty → 0
           && fEq(captured.lonStd, 0.0f, 1e-6)  // empty → 0
           && fEq(captured.hgtStd, 1.0f, 1e-4)
           && captured.numSvs == 8;
}

// 仅原始回调（无 frame 回调）：不应崩溃
bool testBGPRawCallbackOnly() {
    bool                        rawOk = false;
    Novatel::BestGnssPosAParser parser(
        nullptr,
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 0);
        });  // no frameCallback

    auto v = toVecBGP(BESTGNSSPOSA_FRAME1);
    parser.Parse(v.data(), v.size());
    return rawOk;
}

// 粘包：两帧连续放入同一字节流
bool testBGPStickyPacket() {
    int                         callCount = 0;
    Novatel::BestGnssPosAParser parser(
        [&](const Novatel::BestGnssPosA &) {
            callCount++;
        },
        nullptr);

    std::vector<uint8_t> combined;
    for(char c : std::string(BESTGNSSPOSA_FRAME1)) {
        combined.push_back(c);
    }
    for(char c : std::string(BESTGNSSPOSA_FRAME2)) {
        combined.push_back(c);
    }
    parser.Parse(combined.data(), combined.size());

    return callCount == 2;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "\n=== test_bestgnssposa ===\n\n";
    CHECK(testBGPSingleFrame());
    CHECK(testBGPDiffFrame());
    CHECK(testBGPInsufficientObs());
    CHECK(testBGPByteByByte());
    CHECK(testBGPWithGarbage());
    CHECK(testBGPMultipleFrames());
    CHECK(testBGPEmptyFields());
    CHECK(testBGPRawCallbackOnly());
    CHECK(testBGPStickyPacket());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
