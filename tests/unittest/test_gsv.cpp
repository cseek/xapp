#include "test_utils.h"
#include "nmea/gsv.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────
// $GPGSV,totalMsg,msgNum,totalSat,prn,elev,azim,snr,...*hh\r\n
// 2条 GPGSV 构成一轮完整视野（8颗卫星）
static const char GPGSV_FRAME1[] = "$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75\r\n";
static const char GPGSV_FRAME2[] = "$GPGSV,2,2,08,15,07,135,42,16,24,319,38,25,30,088,44,26,12,179,40*7A\r\n";

// $BDGSV：3 颗卫星，最后一条帧只有 3 条卫星条目（不满 4 条）
static const char BDGSV_FRAME[] = "$BDGSV,1,1,03,01,75,180,42,02,45,090,38,03,30,270,35*55\r\n";

// 含空 SNR 字段（未跟踪卫星）
static const char GPGSV_NOSNR_FRAME[] = "$GPGSV,1,1,02,05,10,120,,08,55,280,43*79\r\n";

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（GPGSV）：验证 talkerId、消息信息、所有卫星字段
bool testGsvSingleFrame() {
    Nmea::Gsv captured{};
    bool      rawOk = false, frameOk = false;

    Nmea::GsvParser parser(
        [&](const Nmea::Gsv &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 6) && (memcmp(data, "$GPGSV", 6) == 0);
        });

    auto v = toVec(GPGSV_FRAME1);
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && strcmp(captured.talkerId, "GP") == 0
           && captured.totalMessages == 2
           && captured.messageNumber == 1
           && captured.satellitesInView == 8
           && captured.satelliteCount == 4
           // 第 1 颗卫星
           && captured.satellites[0].prn == 1
           && captured.satellites[0].elevation == 40
           && captured.satellites[0].azimuth == 83
           && captured.satellites[0].snr == 46
           // 第 4 颗卫星
           && captured.satellites[3].prn == 14
           && captured.satellites[3].elevation == 22
           && captured.satellites[3].azimuth == 228
           && captured.satellites[3].snr == 45;
}

// 多 Talker ID：GP 和 BD 各一帧，验证 talkerId 均正确提取
bool testGsvMultipleTalkerIds() {
    int         idx       = 0;
    bool        allOk     = true;
    const char *talkers[] = {"GP", "BD"};

    Nmea::GsvParser parser(
        [&](const Nmea::Gsv &f) {
            if(idx < 2) {
                allOk = allOk && strcmp(f.talkerId, talkers[idx]) == 0;
                idx++;
            }
        },
        nullptr);

    auto v1 = toVec(GPGSV_FRAME1);
    auto v2 = toVec(BDGSV_FRAME);
    parser.Parse(v1.data(), v1.size());
    parser.Parse(v2.data(), v2.size());
    return allOk && idx == 2;
}

// 不满 4 颗卫星的帧：BDGSV 只有 3 颗，验证 satelliteCount == 3
bool testGsvPartialSatellites() {
    Nmea::Gsv       captured{};
    Nmea::GsvParser parser(
        [&](const Nmea::Gsv &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(BDGSV_FRAME);
    parser.Parse(v.data(), v.size());

    return strcmp(captured.talkerId, "BD") == 0
           && captured.satellitesInView == 3
           && captured.satelliteCount == 3
           && captured.satellites[0].prn == 1
           && captured.satellites[2].prn == 3;
}

// 空 SNR 字段（未跟踪卫星）应被置为 -1
bool testGsvEmptySnr() {
    Nmea::Gsv       captured{};
    Nmea::GsvParser parser(
        [&](const Nmea::Gsv &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPGSV_NOSNR_FRAME);
    parser.Parse(v.data(), v.size());

    return captured.satelliteCount == 2
           && captured.satellites[0].prn == 5
           && captured.satellites[0].snr == -1  // 空 SNR
           && captured.satellites[1].prn == 8
           && captured.satellites[1].snr == 43;
}

// 粘包：两帧直接拼接，期望 2 次回调
bool testGsvStickyPacket() {
    int             count = 0;
    Nmea::GsvParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPGSV_FRAME1);
    vecAppend(buf, toVec(GPGSV_FRAME2));

    parser.Parse(buf.data(), buf.size());
    return count == 2;
}

// 分包（逐字节）
bool testGsvSplitPacketByte() {
    bool            rawOk = false;
    Nmea::GsvParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });

    for(uint8_t b : toVec(GPGSV_FRAME1)) {
        parser.ParseByteStream(b);
    }
    return rawOk;
}

// 分包（三段切割）
bool testGsvSplitPacketChunks() {
    int             count = 0;
    Nmea::GsvParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPGSV_FRAME1);
    vecAppend(buf, toVec(GPGSV_FRAME2));

    uint32_t c1 = static_cast<uint32_t>(buf.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(buf.size() * 2 / 3);
    parser.Parse(buf.data(), c1);
    parser.Parse(buf.data() + c1, c2 - c1);
    parser.Parse(buf.data() + c2, static_cast<uint32_t>(buf.size()) - c2);
    return count == 2;
}

// 垃圾数据：帧前后插入含假头的噪声，仍能识别有效帧
bool testGsvWithGarbage() {
    int             count = 0;
    Nmea::GsvParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    // '$','G','P','G','S',<非'V'> 让状态机行进到 SENTENCE3 后 reset
    const uint8_t        garbage[] = {0xFF, 0x01, '$', 'G', 'P', 'G', 'S', 0x01};
    std::vector<uint8_t> stream;
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GPGSV_FRAME1));
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GPGSV_FRAME2));

    parser.Parse(stream.data(), static_cast<uint32_t>(stream.size()));
    return count == 2;
}

// 部分空字段：elevation 和 azimuth 为空（设备仅知 PRN，未计算角度）
// 验证字段索引不错位，空字段归零，snr 仍被正确提取
bool testGsvPartialEmptyFields() {
    // sat[0]: prn=5, elev=空, azim=空, snr=空  → elev/azim/snr = 0/0/-1
    // sat[1]: prn=8, elev=空, azim=280, snr=43 → elev=0, azim=280, snr=43
    static const char PARTIAL_FRAME[] = "$GPGSV,1,1,02,05,,,,08,,280,43*4B\r\n";

    Nmea::Gsv       captured{};
    bool            frameOk = false;
    Nmea::GsvParser parser(
        [&](const Nmea::Gsv &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVec(PARTIAL_FRAME);
    parser.Parse(v.data(), v.size());

    return frameOk
           && captured.satelliteCount == 2
           // sat[0]: 只有 PRN，其余全空
           && captured.satellites[0].prn == 5
           && captured.satellites[0].elevation == 0
           && captured.satellites[0].azimuth == 0
           && captured.satellites[0].snr == -1
           // sat[1]: elev 空，azim/snr 有值
           && captured.satellites[1].prn == 8
           && captured.satellites[1].elevation == 0
           && captured.satellites[1].azimuth == 280
           && captured.satellites[1].snr == 43;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_gsv ===\n\n";
    CHECK(testGsvSingleFrame());
    CHECK(testGsvMultipleTalkerIds());
    CHECK(testGsvPartialSatellites());
    CHECK(testGsvEmptySnr());
    CHECK(testGsvPartialEmptyFields());
    CHECK(testGsvStickyPacket());
    CHECK(testGsvSplitPacketByte());
    CHECK(testGsvSplitPacketChunks());
    CHECK(testGsvWithGarbage());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
