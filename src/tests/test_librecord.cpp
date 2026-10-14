// record_test — validates librecord:
//   Part A (no hardware, always runs; synthetic videotestsrc sources):
//     v0.3 single-file pipeline strings + ASS lines unchanged (golden), segment
//     naming, segmented MJPEG/H.264 (gapless, t=0 starts, per-segment sidecars
//     covering each segment's start), EOS before the first frame (no abort),
//     stall watchdog, mid-stream source error still finalises, loop-overwrite
//     retention safety, luma tap for software auto-exposure (also under a
//     RecordFps cap), splitNow, stop()'s hard deadline covering a worker
//     blocked on sidecar I/O, the periodic disk sync, and the compressed
//     preview tap (rate, timestamps, RecordFps independence, recorded rate
//     unchanged with any mix of taps and source caps, slow/failing consumers,
//     a failure just before stop(), no call after stop()).
//   Part B (first USB camera; SKIPPED when none):
//     1. Strict precompressed gate: a raw format must be REJECTED.
//     2. UVC compressed passthrough + ASS sidecar + live-stream tap.
//     3. Sidecar structure.  4. Stop idempotent / restart.  5. Stale -> dashes.
//     6-8. Per-source validity, ageing and heading range.
//     9. Live segmented recording on the real camera.
//
// Usage: record_test [seconds]   (default 10; Part B recording length)

#include "libcamera.h"
#include "libconfig.h"
#include "liblog.h"
#include "libnetwork.h"
#include "librecord.h"

#include <linux/videodev2.h>

#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <ctime>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace dashcam::camera;
namespace fs  = std::filesystem;
namespace net = dashcam::network;
namespace rec = dashcam::record;

namespace dashcam::record {
// Test-only access: replace the v4l2src head with a synthetic source and reach
// the pipeline to inject faults.
struct RecorderTestHook {
    static void setSource(SegmentedRecorder& r, std::string desc) { r.testSourceDesc_ = std::move(desc); }
    static GstElement* pipeline(SegmentedRecorder& r) { return r.pipeline_; }
    static void setSyncHook(SegmentedRecorder& r, std::function<void()> fn) { r.syncTestHook_ = std::move(fn); }
    static void setWorkerHook(SegmentedRecorder& r, std::function<void()> fn) { r.workerTestHook_ = std::move(fn); }
    static void closePreviewGate(SegmentedRecorder& r) { r.previewGate_.store(false); }  ///< As stop() does first.
};
} // namespace dashcam::record

static int  g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::cout << (ok ? "  ok    " : "  FAIL  ") << what << "\n";
    if (!ok) ++g_fails;
}

// Read from a connected socket until the peer goes idle or the window elapses.
static std::string drainSock(net::TcpSocket& s, int totalMs) {
    std::string acc;
    char buf[8192];
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(totalMs);
    while (std::chrono::steady_clock::now() < end) {
        size_t got = 0;
        net::IoStatus st = s.recv(buf, sizeof(buf), 150, got);
        if (st == net::IoStatus::Ok)          acc.append(buf, got);
        else if (st == net::IoStatus::Timeout) { if (!acc.empty()) break; }
        else                                   break;
    }
    return acc;
}

static int64_t epochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// ─── Part A helpers ───────────────────────────────────────────────────────────

static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static std::string makeTempDir(const char* tag) {
    std::string tmpl = std::string("/tmp/record_test_") + tag + "_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    const char* d = ::mkdtemp(buf.data());
    return d ? std::string(d) : std::string();
}

// Demux an MKV in-process: frame count and first/last PTS.  ok = clean EOS.
struct MkvInfo {
    bool    ok = false;
    int     frames = 0;
    int64_t firstPts = -1, lastPts = -1;
    std::set<uint64_t>* hashes = nullptr;   ///< When set: FNV-1a of every frame's bytes.
};

static uint64_t fnv1a(const uint8_t* p, size_t n) {
    uint64_t h = 1469598103934665603ULL;
    for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}

static GstPadProbeReturn countProbe(GstPad*, GstPadProbeInfo* info, gpointer user) {
    auto* mi = static_cast<MkvInfo*>(user);
    GstBuffer* b = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!b) return GST_PAD_PROBE_OK;
    ++mi->frames;
    GstMapInfo map;
    if (mi->hashes && gst_buffer_map(b, &map, GST_MAP_READ)) {
        mi->hashes->insert(fnv1a(map.data, map.size));
        gst_buffer_unmap(b, &map);
    }
    if (GST_BUFFER_PTS_IS_VALID(b)) {
        const int64_t pts = static_cast<int64_t>(GST_BUFFER_PTS(b));
        if (mi->firstPts < 0 || pts < mi->firstPts) mi->firstPts = pts;
        if (pts > mi->lastPts) mi->lastPts = pts;
    }
    return GST_PAD_PROBE_OK;
}

static MkvInfo probeMkv(const std::string& path, std::set<uint64_t>* hashes = nullptr) {
    MkvInfo mi;
    mi.hashes = hashes;
    const std::string desc = "filesrc location=" + rec::detail::gstQuoted(path) +
                             " ! matroskademux ! fakesink name=sink sync=false";
    GError* err = nullptr;
    GstElement* p = gst_parse_launch(desc.c_str(), &err);
    if (!p || err) { if (err) g_error_free(err); if (p) gst_object_unref(p); return mi; }
    GstElement* sink = gst_bin_get_by_name(GST_BIN(p), "sink");
    GstPad* pad = gst_element_get_static_pad(sink, "sink");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, countProbe, &mi, nullptr);
    gst_object_unref(pad);
    gst_object_unref(sink);
    gst_element_set_state(p, GST_STATE_PLAYING);
    GstBus* bus = gst_element_get_bus(p);
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus, 20 * GST_SECOND, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    mi.ok = msg && GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS;
    if (msg) gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(p, GST_STATE_NULL);
    gst_object_unref(p);
    return mi;
}

// Parse "H:MM:SS.CC" → ns.
static int64_t parseAssTime(const std::string& t) {
    int h = 0, m = 0, sec = 0, cs = 0;
    if (std::sscanf(t.c_str(), "%d:%d:%d.%d", &h, &m, &sec, &cs) != 4) return -1;
    return ((int64_t(h) * 3600 + m * 60 + sec) * 100 + cs) * 10000000LL;
}

struct AssInfo {
    bool    exists = false;
    int     clockEvents = 0;      ///< BR (clock) events = samples.
    int64_t firstStartNs = -1;    ///< Start time of the first sample.
    int     spdDash = 0, spdValue = 0, adas = 0, styles = 0;
};

static AssInfo readAss(const std::string& path) {
    AssInfo ai;
    std::ifstream in(path);
    if (!in) return ai;
    ai.exists = true;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("Style: ", 0) == 0) ++ai.styles;
        if (line.rfind("Dialogue: 0,", 0) != 0) continue;
        if (line.find(",BR,,0,0,0,,") != std::string::npos) {
            ++ai.clockEvents;
            const int64_t t = parseAssTime(line.substr(12, line.find(',', 12) - 12));
            if (t >= 0 && (ai.firstStartNs < 0 || t < ai.firstStartNs)) ai.firstStartNs = t;
        }
        if (line.find(",TL,,0,0,0,,SPD -- km/h") != std::string::npos) ++ai.spdDash;
        else if (line.find(",TL,,0,0,0,,SPD ") != std::string::npos) ++ai.spdValue;
        if (line.find(",ADAS,,") != std::string::npos) ++ai.adas;
    }
    return ai;
}

// Owned segment files in @p dir, SEQ-ordered: seq → (mkv path, ass path).
static std::map<uint64_t, std::pair<std::string, std::string>> listSegments(const std::string& dir) {
    std::map<uint64_t, std::pair<std::string, std::string>> out;
    for (const auto& e : fs::directory_iterator(dir)) {
        bool isAss = false;
        const auto seq = rec::parseSegmentName(e.path().filename().string(), "dashcam", &isAss);
        if (!seq) continue;
        (isAss ? out[*seq].second : out[*seq].first) = e.path().string();
    }
    return out;
}

// Keep the overlay fresh (clock only, no position) while a session runs.
static void feedClockOnly(rec::SegmentedRecorder& r, int ms) {
    for (int t = 0; t < ms / 100; ++t) {
        rec::OverlayData od;
        od.timestampMs   = epochMs();
        od.positionValid = false;
        r.setOverlayData(od);
        sleepMs(100);
    }
}

static const char* kMjpegSrc =
    "videotestsrc is-live=true pattern=ball ! video/x-raw,width=320,height=240,framerate=30/1 "
    "! jpegenc";

static rec::RecordingFormat fmt320(uint32_t fourcc) {
    rec::RecordingFormat f;
    f.v4l2PixFmt = fourcc;
    f.width = 320; f.height = 240; f.fps = 30.0f;
    return f;
}

static rec::SegmentOptions segOpts(const std::string& dir, uint32_t segSec) {
    rec::SegmentOptions o;
    o.dir = dir;
    o.segmentSec = segSec;
    o.stallTimeoutMs = 2000;
    o.firstFrameTimeoutMs = 3000;
    o.exitOnTeardownHang = false;
    return o;
}

// ─── A1: v0.3 single-file path unchanged ──────────────────────────────────────

static void testGoldenSingleFile() {
    std::cout << "\n--- A1: v0.3 single-file pipeline + ASS lines unchanged ---\n";
    struct Case { uint32_t fourcc; uint32_t maxFps; bool tap; std::string expect; };
    const std::string dev = "/dev/video_record_test_missing";
    const std::vector<Case> cases = {
        {V4L2_PIX_FMT_MJPEG, 0, false,
         "v4l2src name=camerasrc device=\"" + dev + "\" ! image/jpeg, width=1920, height=1080, "
         "framerate=30/1 ! queue max-size-buffers=8 leaky=0 ! matroskamux offset-to-zero=true "
         "! filesink name=fsink sync=false async=false location=\"/tmp/g.mkv\""},
        {V4L2_PIX_FMT_MJPEG, 15, false,
         "v4l2src name=camerasrc device=\"" + dev + "\" ! image/jpeg, width=1920, height=1080, "
         "framerate=30/1 ! videorate drop-only=true max-rate=15 ! queue max-size-buffers=8 leaky=0 "
         "! matroskamux offset-to-zero=true ! filesink name=fsink sync=false async=false "
         "location=\"/tmp/g.mkv\""},
        {V4L2_PIX_FMT_H264, 15, false,
         "v4l2src name=camerasrc device=\"" + dev + "\" ! video/x-h264, width=1920, height=1080, "
         "framerate=30/1 ! h264parse ! queue max-size-buffers=8 leaky=0 ! matroskamux "
         "offset-to-zero=true ! filesink name=fsink sync=false async=false location=\"/tmp/g.mkv\""},
        {V4L2_PIX_FMT_H264, 0, true,
         "v4l2src name=camerasrc device=\"" + dev + "\" ! video/x-h264, width=1920, height=1080, "
         "framerate=30/1 ! tee name=rectee rectee. ! h264parse ! queue max-size-buffers=8 leaky=0 "
         "! matroskamux offset-to-zero=true ! filesink name=fsink sync=false async=false "
         "location=\"/tmp/g.mkv\" rectee. ! queue max-size-buffers=4 leaky=downstream ! h264parse "
         "config-interval=-1 ! video/x-h264, stream-format=byte-stream, alignment=au ! appsink "
         "name=streamsink emit-signals=false sync=false max-buffers=4 drop=true"},
    };
    int idx = 0;
    for (const auto& c : cases) {
        std::string seen;
        rec::Recorder r;
        r.setLogCallback([&](dashcam::log::LogLevel, const std::string& m) {
            const std::string key = "recording pipeline: ";
            if (m.rfind(key, 0) == 0) seen = m.substr(key.size());
        });
        if (c.tap) r.setCompressedFrameCallback([](const uint8_t*, size_t, bool) {});
        rec::RecordingFormat f;
        f.v4l2PixFmt = c.fourcc; f.width = 1920; f.height = 1080; f.fps = 30.0f;
        const bool started = r.startRecording(dev, f, "/tmp/g.mkv", c.maxFps);
        r.stopRecording();
        check(!started, "golden case " + std::to_string(idx) + ": missing device refused");
        // librecord's log helper truncates the whole message at 511 chars.
        const std::string want = c.expect.substr(0, 511 - std::strlen("recording pipeline: "));
        check(seen == want, "golden case " + std::to_string(idx) + ": pipeline string unchanged");
        if (seen != want) std::cout << "    got:    " << seen << "\n    expect: " << c.expect << "\n";
        ++idx;
    }
    fs::remove("/tmp/g.mkv");
    fs::remove("/tmp/g.ass");

    // Sidecar line format: every source fresh.
    rec::OverlayData od;          // defaults: 10.7725 N, 106.6581 E, 52.3 m, 1.5 km/h, 90 deg
    std::ostringstream fresh;
    rec::detail::writeAssSample(fresh, 1'230'000'000LL, 200'000'000LL, od, 0,
                                false, false, false, false);
    const std::string f = fresh.str();
    check(f.find("Dialogue: 0,0:00:01.23,0:00:01.43,TL,,0,0,0,,SPD 1.5 km/h\\NACC +0.0 m/s2\n") == 0,
          "fresh TL line unchanged");
    check(f.find(",TR,,0,0,0,,HDG 090 E\n") != std::string::npos, "fresh TR line unchanged");
    check(f.find(",BL,,0,0,0,,LAT 10.772500 N\\NLON 106.658100 E\\NALT 52.3 m\n") != std::string::npos,
          "fresh BL line unchanged");
    check(f.find(",ADAS,,") == std::string::npos, "no ADAS banner by default");
    // A 2-D fix: position live, altitude NaN → "ALT --", not "ALT nan m".
    rec::OverlayData od2d;
    od2d.positionValid = true;
    od2d.altitudeM = std::nan("");
    std::ostringstream twoD;
    rec::detail::writeAssSample(twoD, 0, 200'000'000LL, od2d, 0, false, false, false, false);
    check(twoD.str().find(",BL,,0,0,0,,LAT 10.772500 N\\NLON 106.658100 E\\NALT --\n") != std::string::npos,
          "fix without altitude: ALT dashed on its own");
    // v0.4's clock-only snapshot: no source valid (the defaults) → all dashed.
    const rec::OverlayData none;
    const auto st = rec::detail::assStaleness(none, 0, 2000);
    check(st.speed && st.accel && st.position && st.heading, "default snapshot: every source stale");
    std::ostringstream nopos;
    rec::detail::writeAssSample(nopos, 0, 200'000'000LL, none, 0, st.speed, st.accel,
                                st.position, st.heading);
    check(nopos.str().find(",TL,,0,0,0,,SPD -- km/h\\NACC -- m/s2") != std::string::npos &&
          nopos.str().find(",TR,,0,0,0,,HDG --") != std::string::npos &&
          nopos.str().find("LAT --\\NLON --\\NALT --") != std::string::npos &&
          nopos.str().find(",BR,,0,0,0,,") != std::string::npos,
          "clock-only snapshot renders dashes, clock kept");

    // Detail block: absent unless the application supplies text.
    check(f.find(",DET,,") == std::string::npos, "no detail block without detailText");
    rec::OverlayData odd;
    odd.detailText = "VSS 52.34 CAN  RPM 1200\nBRAKE off, {\\b1}x";
    odd.detailTimestampMs = 10'000;
    std::ostringstream det;
    rec::detail::writeAssSample(det, 0, 200'000'000LL, odd, 10'500, true, true, true, true, false);
    check(det.str().find(",DET,,0,0,0,,VSS 52.34 CAN  RPM 1200\\NBRAKE off; (/b1)x\n") != std::string::npos,
          "detail block: lines joined with \\N, comma/brace/backslash replaced");
    const auto stOld = rec::detail::assStaleness(odd, 10'000 + 2'001, 2000);
    const auto stNew = rec::detail::assStaleness(odd, 10'000 + 2'000, 2000);
    check(stOld.detail && !stNew.detail, "detail stale past StaleTimeoutMs only");
    check(!rec::detail::assStaleness(odd, 99'000'000, 0).detail, "StaleTimeoutMs 0: detail never stale");
    std::ostringstream detStale;
    rec::detail::writeAssSample(detStale, 0, 200'000'000LL, odd, 12'500, true, true, true, true, stOld.detail);
    check(detStale.str().find(",DET,,0,0,0,,TLM --\n") != std::string::npos &&
          detStale.str().find("VSS") == std::string::npos, "stale detail block becomes TLM --, old text gone");
    std::ostringstream hdr;
    rec::detail::writeAssHeader(hdr, dashcam::config::OverlayConfig{}, 1920, 1080);
    const std::string h = hdr.str();
    const size_t dpos = h.find("Style: DET,");
    const std::string detStyle = dpos == std::string::npos ? "" : h.substr(dpos, h.find('\n', dpos) - dpos);
    // ... BorderStyle 3, Outline n, Shadow 0, Alignment 4 (middle-left), margins, encoding 1
    check(detStyle.find(",0,0,3,3,0,4,18,18,12,1") != std::string::npos,
          "header defines the DET style (middle-left, box): " + detStyle);
}

// ─── A2: segment naming ───────────────────────────────────────────────────────

static void testNaming() {
    std::cout << "\n--- A2: segment naming ---\n";
    bool isAss = true;
    auto p = rec::parseSegmentName("dashcam_000042_20260923_120000.mkv", "dashcam", &isAss);
    check(p && *p == 42 && !isAss, "parses a .mkv segment");
    p = rec::parseSegmentName("dashcam_000042_20260923_120000.ass", "dashcam", &isAss);
    check(p && *p == 42 && isAss, "parses its .ass sidecar");
    check(rec::parseSegmentName("dashcam_123456789012_20260923_120000.mkv", "dashcam").value_or(0) ==
              123456789012ULL, "12-digit SEQ accepted");
    const char* bad[] = {
        "dashcam_00042_20260923_120000.mkv",        // 5-digit SEQ
        "dashcam_1234567890123_20260923_120000.mkv",// 13-digit SEQ
        "dashcamx_000042_20260923_120000.mkv",      // other prefix
        "primary_20260923_120000.mkv",              // v0.3 footage
        "dashcam_000042_20260923_120000.mkv.tmp",
        "dashcam_000042_20260923_120000.MKV",
        "dashcam_000042_2026092_120000.mkv",
        "dashcam_000042_20260923-120000.mkv",
        "dashcam_00004a_20260923_120000.mkv",
        "dashcam_000042_20260923_12000a.mkv",
        "dashcam_000042.mkv",
        "dashcam_.mkv",
        "",
    };
    int rejected = 0;
    for (const char* b : bad) if (!rec::parseSegmentName(b, "dashcam")) ++rejected;
    check(rejected == (int)(sizeof(bad) / sizeof(bad[0])), "foreign names rejected (" +
          std::to_string(rejected) + "/" + std::to_string(sizeof(bad) / sizeof(bad[0])) + ")");
    const std::string made = rec::segmentFileName("dashcam", 7, 0);
    check(rec::parseSegmentName(made, "dashcam").value_or(0) == 7, "segmentFileName round-trips (" + made + ")");

    const std::string dir = makeTempDir("seq");
    check(rec::nextSegmentSeq(dir, "dashcam") == 1, "empty dir starts at SEQ 1");
    std::ofstream(dir + "/dashcam_000009_20200101_000000.mkv") << "x";
    std::ofstream(dir + "/dashcam_000003_20300101_000000.ass") << "x";
    std::ofstream(dir + "/dashcam_999999999_bogus.mkv") << "x";
    check(rec::nextSegmentSeq(dir, "dashcam") == 10, "next SEQ = highest owned + 1");
    check(rec::nextSegmentSeq("/nonexistent/record_test", "dashcam") == 1, "unreadable dir -> 1");
    fs::remove_all(dir);
}

// ─── A3/A4: segmented MJPEG + H.264 ───────────────────────────────────────────

static void testSegmented(const char* label, const std::string& srcDesc, uint32_t fourcc) {
    std::cout << "\n--- " << label << " ---\n";
    const std::string dir = makeTempDir("seg");
    std::ofstream(dir + "/dashcam_000004_20200101_000000.mkv") << "old";   // continue after SEQ 4
    std::ofstream(dir + "/primary_20200101_000000.mkv") << "foreign";

    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    r.setOverlayConfig(dashcam::config::OverlayConfig{});
    rec::RecorderTestHook::setSource(r, srcDesc);
    check(r.deletableBelowSeq() == UINT64_MAX, "idle watermark = everything");
    const bool started = r.start("unused", fmt320(fourcc), segOpts(dir, 2));
    check(started, "start OK");
    if (!started) { std::cout << "    " << r.lastError() << "\n"; fs::remove_all(dir); return; }

    feedClockOnly(r, 1500);
    const uint64_t wm = r.deletableBelowSeq();
    check(wm == 5, "watermark protects the first open segment (" + std::to_string(wm) + ")");
    feedClockOnly(r, 5500);
    check(r.isRecording(), "healthy throughout");
    check(r.consumeFragmentClosed(), "fragment-closed observed");
    check(r.deletableBelowSeq() > 5, "closed segments become deletable");
    const bool clean = r.stop();
    const uint64_t frames = r.framesReceived();
    check(clean, "stop() finalised cleanly");
    check(r.deletableBelowSeq() == UINT64_MAX, "watermark released after stop");

    const auto segs = listSegments(dir);
    std::vector<uint64_t> seqs;
    for (const auto& [seq, files] : segs) if (seq >= 5) seqs.push_back(seq);
    check(seqs.size() >= 3, std::to_string(seqs.size()) + " new segments (>= 3)");
    bool consecutive = !seqs.empty() && seqs.front() == 5;
    for (size_t i = 1; i < seqs.size(); ++i) consecutive = consecutive && seqs[i] == seqs[i - 1] + 1;
    check(consecutive, "SEQs consecutive from 5");

    int total = 0;
    bool allOk = true, allZero = true, allAss = true, allCovered = true, dense = true, dashes = true;
    for (uint64_t seq : seqs) {
        const auto& [mkv, ass] = segs.at(seq);
        const MkvInfo mi = probeMkv(mkv);
        const AssInfo ai = readAss(ass);
        total += mi.frames;
        allOk   = allOk && mi.ok && mi.frames > 0;
        allZero = allZero && mi.firstPts >= 0 && mi.firstPts < 100'000'000LL;
        allAss  = allAss && ai.exists && ai.styles == 6;
        const double durS = (mi.lastPts - mi.firstPts) / 1e9 + 1.0 / 30;
        // Sidecar must cover the segment from its very start (first GOP replay)
        // and carry most of the ideal 5 Hz samples.
        allCovered = allCovered && ai.firstStartNs >= 0 && ai.firstStartNs < 500'000'000LL;
        if (durS > 1.5) dense = dense && ai.clockEvents >= (int)(0.8 * durS * 5);
        dashes = dashes && ai.spdValue == 0 && ai.spdDash == ai.clockEvents && ai.adas == 0;
        std::cout << "    seq " << seq << ": " << mi.frames << " frames, first pts "
                  << mi.firstPts / 1000000 << " ms, " << durS << " s, " << ai.clockEvents
                  << " sidecar samples, first at " << ai.firstStartNs / 1000000 << " ms\n";
    }
    check(allOk, "every segment demuxes to EOS with frames");
    check(allZero, "every segment starts at t~0");
    check(allAss, "every segment has its own .ass (6 styles)");
    check(allCovered, "every sidecar covers its segment start (< 0.5 s)");
    check(dense, "sidecar density >= 80% of 5 Hz in every full segment");
    check(dashes, "clock-only sidecar: SPD/position dashed, no ADAS banner");
    check(total > 0 && (uint64_t)total == frames,
          "gapless: frames in files (" + std::to_string(total) + ") == frames captured (" +
          std::to_string(frames) + ")");
    check(fs::exists(dir + "/primary_20200101_000000.mkv"), "foreign file untouched");
    fs::remove_all(dir);
}

// ─── A5: EOS before the first buffer must not abort the process ───────────────

static void testEosBeforeFirstBuffer() {
    std::cout << "\n--- A5: EOS before the first frame (splitmuxsink abort guard) ---\n";
    const std::string dir = makeTempDir("eos");
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    rec::RecorderTestHook::setSource(
        r, "videotestsrc num-buffers=0 ! video/x-raw,width=320,height=240,framerate=30/1 ! jpegenc");
    auto o = segOpts(dir, 2);
    o.firstFrameTimeoutMs = 1500;
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), "start OK");
    sleepMs(2500);
    check(!r.isRecording(), "first-frame timeout marks the session unhealthy");
    check(r.lastError().find("first frame") != std::string::npos, "reason: " + r.lastError());
    check(r.stop(), "stop() returns (process not aborted)");
    check(listSegments(dir).empty(), "no segment files created");
    fs::remove_all(dir);
}

// ─── A6: stall watchdog ───────────────────────────────────────────────────────

static GstPadProbeReturn dropAll(GstPad*, GstPadProbeInfo*, gpointer) { return GST_PAD_PROBE_DROP; }

static void testStall() {
    std::cout << "\n--- A6: stall watchdog ---\n";
    const std::string dir = makeTempDir("stall");
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    rec::RecorderTestHook::setSource(r, std::string(kMjpegSrc) + " ! identity name=gate");
    auto o = segOpts(dir, 60);
    o.stallTimeoutMs = 1000;
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), "start OK");
    feedClockOnly(r, 2000);
    check(r.isRecording(), "healthy while frames flow");
    GstElement* gate = gst_bin_get_by_name(GST_BIN(rec::RecorderTestHook::pipeline(r)), "gate");
    GstPad* pad = gst_element_get_static_pad(gate, "src");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, dropAll, nullptr, nullptr);
    gst_object_unref(pad);
    gst_object_unref(gate);
    sleepMs(2000);
    check(!r.isRecording(), "stall detected");
    check(r.lastError().find("stalled") != std::string::npos, "reason: " + r.lastError());
    check(r.stop(), "stop() finalises after a stall");
    const auto segs = listSegments(dir);
    const MkvInfo mi = segs.empty() ? MkvInfo{} : probeMkv(segs.begin()->second.first);
    check(segs.size() == 1 && mi.ok && mi.frames >= 45, "stalled segment playable (" +
          std::to_string(mi.frames) + " frames)");
    fs::remove_all(dir);
}

// ─── A7: mid-stream source error still finalises the open segment ────────────

static void testSourceError() {
    std::cout << "\n--- A7: source error mid-segment (unplug stand-in) ---\n";
    const std::string dir = makeTempDir("err");
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    rec::RecorderTestHook::setSource(r, std::string(kMjpegSrc) + " ! identity error-after=60");
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)), "start OK");
    feedClockOnly(r, 3000);
    check(!r.isRecording(), "error marks the session unhealthy");
    check(r.lastError().find("pipeline error") != std::string::npos, "reason: " + r.lastError());
    const bool clean = r.stop();
    check(clean, "stop() finalised the segment via EOS injected at the record queue");
    const auto segs = listSegments(dir);
    const MkvInfo mi = segs.empty() ? MkvInfo{} : probeMkv(segs.begin()->second.first);
    check(segs.size() == 1 && mi.ok && mi.frames >= 55, "errored segment playable (" +
          std::to_string(mi.frames) + " frames)");
    fs::remove_all(dir);
}

// ─── A8: loop-overwrite retention ─────────────────────────────────────────────

static void writeBytes(const std::string& path, size_t n) {
    std::ofstream f(path, std::ios::binary);
    std::string chunk(n, 'x');
    f.write(chunk.data(), (std::streamsize)chunk.size());
}

static void testRetention() {
    std::cout << "\n--- A8: loop-overwrite retention ---\n";
    const size_t K = 64 * 1024;   // 64 KiB files -> exact st_blocks accounting
    auto seg = [](uint64_t seq, const char* stamp, const char* ext) {
        char b[96];
        std::snprintf(b, sizeof(b), "dashcam_%06llu_%s.%s", (unsigned long long)seq, stamp, ext);
        return std::string(b);
    };
    auto keep = [] { return true; };
    auto log = dashcam::log::getCallback();

    // (1) SEQ order beats the clock; quota respected; watermark protects.
    {
        const std::string d = makeTempDir("ret1");
        writeBytes(d + "/" + seg(1, "20300101_000000", "mkv"), K);   // oldest SEQ, NEWEST clock
        writeBytes(d + "/" + seg(1, "20300101_000000", "ass"), K);
        writeBytes(d + "/" + seg(2, "20200101_000000", "mkv"), K);
        writeBytes(d + "/" + seg(3, "20100101_000000", "mkv"), K);
        writeBytes(d + "/" + seg(4, "20000101_000000", "mkv"), K);   // "active"
        rec::RetentionPolicy p;
        p.dir = d;
        p.maxBytes = 3 * K;
        auto res = rec::enforceRetention(p, 4, keep, log);
        check(!fs::exists(d + "/" + seg(1, "20300101_000000", "mkv")) &&
              !fs::exists(d + "/" + seg(1, "20300101_000000", "ass")),
              "lowest SEQ unit (mkv+ass) deleted first despite the newest clock");
        check(fs::exists(d + "/" + seg(2, "20200101_000000", "mkv")) && res.deleted == 2 && !res.stillOver,
              "stops as soon as the quota holds");
        p.maxBytes = 1;
        res = rec::enforceRetention(p, 4, keep, log);
        check(fs::exists(d + "/" + seg(4, "20000101_000000", "mkv")) && res.stillOver,
              "watermark protects the active segment even over quota");
        check(!fs::exists(d + "/" + seg(3, "20100101_000000", "mkv")), "everything below the watermark went");
        fs::remove_all(d);
    }
    // (2) Foreign files, symlinks and directories are never touched.
    {
        const std::string d = makeTempDir("ret2");
        const std::string outside = makeTempDir("ret2_target");
        writeBytes(outside + "/precious.mkv", K);
        writeBytes(d + "/primary_20260101_000000.mkv", K);
        writeBytes(d + "/notes.txt", K);
        writeBytes(d + "/dashcam_1_20260101_000000.mkv", K);
        fs::create_symlink(outside + "/precious.mkv", d + "/" + seg(1, "20200101_000000", "mkv"));
        fs::create_directory(d + "/" + seg(2, "20200101_000000", "mkv"));
        writeBytes(d + "/" + seg(3, "20200101_000000", "ass"), K);      // orphan .ass
        writeBytes(d + "/" + seg(5, "20200101_000000", "mkv"), K);
        rec::RetentionPolicy p;
        p.dir = d;
        p.maxBytes = 1;
        const auto res = rec::enforceRetention(p, UINT64_MAX, keep, log);
        check(fs::exists(d + "/primary_20260101_000000.mkv") && fs::exists(d + "/notes.txt") &&
              fs::exists(d + "/dashcam_1_20260101_000000.mkv"), "foreign files untouched");
        check(fs::is_symlink(d + "/" + seg(1, "20200101_000000", "mkv")) &&
              fs::exists(outside + "/precious.mkv"), "symlink (and its target) untouched");
        check(fs::is_directory(d + "/" + seg(2, "20200101_000000", "mkv")), "directory untouched");
        check(!fs::exists(d + "/" + seg(3, "20200101_000000", "ass")), "orphan .ass deleted");
        check(!fs::exists(d + "/" + seg(5, "20200101_000000", "mkv")) && res.errors == 0 &&
              res.ownedBytes == 0, "all owned segments deleted, owned bytes 0");
        fs::remove_all(d);
        fs::remove_all(outside);
    }
    // (3) Free-space floor, termination, keepGoing, nothing-to-do.
    {
        const std::string d = makeTempDir("ret3");
        for (uint64_t s = 1; s <= 5; ++s) writeBytes(d + "/" + seg(s, "20200101_000000", "mkv"), K);
        rec::RetentionPolicy p;
        p.dir = d;
        p.maxBytes = 100 * K;
        auto res = rec::enforceRetention(p, UINT64_MAX, keep, log);
        check(res.deleted == 0 && !res.stillOver && res.ownedBytes == 5 * K && res.freeBytes > 0,
              "within quota and floor: nothing deleted");
        int calls = 0;
        p.minFreeBytes = UINT64_MAX / 2;                      // unreachable floor
        res = rec::enforceRetention(p, UINT64_MAX, [&] { return ++calls <= 2; }, log);
        check(res.deleted == 2, "keepGoing=false stops the pass (2 deleted)");
        res = rec::enforceRetention(p, UINT64_MAX, keep, log);
        check(res.deleted == 3 && res.stillOver, "unreachable floor: deletes all, terminates, reports stillOver");
        res = rec::enforceRetention(p, UINT64_MAX, keep, log);
        check(res.deleted == 0 && res.stillOver, "empty dir: terminates immediately");
        fs::remove_all(d);
        const auto miss = rec::enforceRetention(p, UINT64_MAX, keep, log);
        check(miss.errors == 1 && miss.deleted == 0, "missing dir reported, no crash");
    }
}

// ─── A9: luma tap (software auto-exposure feed) ───────────────────────────────

static void testLumaTap() {
    std::cout << "\n--- A9: luma tap ---\n";
    struct Case { const char* pattern; float lo, hi; };
    for (const Case& c : {Case{"white", 200.f, 255.f}, Case{"black", 0.f, 40.f}}) {
        const std::string dir = makeTempDir("luma");
        rec::SegmentedRecorder r;
        r.setLogCallback(dashcam::log::getCallback());
        r.setLumaTap(true);
        rec::RecorderTestHook::setSource(
            r, std::string("videotestsrc is-live=true pattern=") + c.pattern +
               " ! video/x-raw,width=320,height=240,framerate=30/1 ! jpegenc");
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)),
              std::string(c.pattern) + ": start with luma tap");
        float luma = -1;
        uint64_t seq = 0;
        check(!r.latestLuma(luma, seq), std::string(c.pattern) + ": no sample before frames flow");
        feedClockOnly(r, 3000);
        const bool got = r.latestLuma(luma, seq);
        check(got && seq >= 4 && seq <= 8, std::string(c.pattern) + ": ~2 samples/s (" +
              std::to_string(seq) + " in 3 s)");
        check(got && luma >= c.lo && luma <= c.hi, std::string(c.pattern) + ": mean luma " +
              std::to_string(int(luma)) + " in [" + std::to_string(int(c.lo)) + "," +
              std::to_string(int(c.hi)) + "]");
        check(r.isRecording(), std::string(c.pattern) + ": recording healthy with the tap");
        check(r.stop(), std::string(c.pattern) + ": stop finalised");
        const uint64_t frames = r.framesReceived();
        const auto segs = listSegments(dir);
        const MkvInfo mi = segs.empty() ? MkvInfo{} : probeMkv(segs.begin()->second.first);
        check(mi.ok && (uint64_t)mi.frames == frames, std::string(c.pattern) +
              ": every frame recorded despite the tap (" + std::to_string(mi.frames) + "/" +
              std::to_string(frames) + ")");
        fs::remove_all(dir);
    }
    // H.264 sources get no tap (it would need a full-rate software decode).
    const std::string dir = makeTempDir("luma264");
    rec::SegmentedRecorder r;
    r.setLumaTap(true);
    rec::RecorderTestHook::setSource(
        r, "videotestsrc is-live=true ! video/x-raw,width=320,height=240,framerate=30/1 "
           "! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30");
    check(r.start("unused", fmt320(V4L2_PIX_FMT_H264), segOpts(dir, 60)), "H.264: start");
    sleepMs(1500);
    float luma = 0;
    uint64_t seq = 0;
    check(!r.latestLuma(luma, seq) && r.isRecording(), "H.264: no tap, recording unaffected");
    r.stop();
    fs::remove_all(dir);
}

// ─── A9b: luma tap under a RecordFps cap ──────────────────────────────────────

// The source kinds the rate tests run on: jpegenc's own caps (what most tests
// use), an explicit fixed-rate caps filter like the real v4l2src head
// ("image/jpeg, width, height, framerate=30/1"), and a source whose rate is
// left to negotiation — the strictest: any caps-constraining element in a tap
// would pull the camera itself down.
struct RateSource { const char* tag; std::string desc; };
static std::vector<RateSource> rateSources(bool withNegotiable) {
    std::vector<RateSource> v = {
        {"jpegenc caps", kMjpegSrc},
        {"fixed caps", std::string(kMjpegSrc) + " ! image/jpeg,width=320,height=240,framerate=30/1"},
    };
    if (withNegotiable)
        v.push_back({"negotiable rate",
                     "videotestsrc is-live=true pattern=ball ! video/x-raw,width=320,height=240 ! jpegenc"});
    return v;
}

// Frames in all owned segments of @p dir; @p hashes collects their bytes' hashes.
static int recordedFrames(const std::string& dir, bool& allOk, std::set<uint64_t>* hashes = nullptr) {
    int total = 0;
    allOk = true;
    for (const auto& [seq, files] : listSegments(dir)) {
        const MkvInfo mi = probeMkv(files.first, hashes);
        total += mi.frames;
        allOk = allOk && mi.ok;
    }
    return total;
}

// The recording's negotiated frame rate (recq's caps), e.g. "5/1".
static std::string recFramerate(rec::SegmentedRecorder& r) {
    std::string out = "none";
    GstElement* q = gst_bin_get_by_name(GST_BIN(rec::RecorderTestHook::pipeline(r)), "recq");
    if (!q) return out;
    GstPad* pad = gst_element_get_static_pad(q, "sink");
    if (GstCaps* caps = gst_pad_get_current_caps(pad)) {
        gint n = 0, d = 0;
        if (gst_caps_get_size(caps) > 0 &&
            gst_structure_get_fraction(gst_caps_get_structure(caps, 0), "framerate", &n, &d))
            out = std::to_string(n) + "/" + std::to_string(d);
        gst_caps_unref(caps);
    }
    gst_object_unref(pad);
    gst_object_unref(q);
    return out;
}

// Steady-state rates of a running session over @p windowMs (after the first
// frame and a 500 ms warm-up), per second.
struct Rates { double rec = 0, source = 0, preview = 0, luma = 0; };
static Rates measureRates(rec::SegmentedRecorder& r, int windowMs) {
    for (int i = 0; i < 60 && r.framesReceived() == 0; ++i) sleepMs(50);
    feedClockOnly(r, 500);
    float    luma = 0;
    uint64_t l0 = 0, l1 = 0;
    r.latestLuma(luma, l0);                               // seq is set (0) even without a sample
    const uint64_t f0 = r.framesReceived(), s0 = r.sourceFramesReceived(), p0 = r.previewFramesTapped();
    const auto t0 = std::chrono::steady_clock::now();
    feedClockOnly(r, windowMs);
    const uint64_t f1 = r.framesReceived(), s1 = r.sourceFramesReceived(), p1 = r.previewFramesTapped();
    r.latestLuma(luma, l1);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    Rates out;
    out.rec     = double(f1 - f0) / secs;
    out.source  = double(s1 - s0) / secs;
    out.preview = double(p1 - p0) / secs;
    out.luma    = double(l1 - l0) / secs;
    return out;
}

static std::string fmtFps(double v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f", v);
    return b;
}

// Regression: lumarate (videorate max-rate=2) used to sit in the luma branch;
// behind the recrate cap its caps negotiation reached through rectee and
// pulled the RECORDING down to 2 fps (6 frames in 3 s at RecordFps 5).
static void testLumaUnderRecordFps() {
    std::cout << "\n--- A9b: luma tap under RecordFps 5 records at 5 fps ---\n";
    for (const RateSource& src : rateSources(false)) {
        const std::string tag = src.tag;
        const std::string dir = makeTempDir("lumacap");
        rec::SegmentedRecorder r;
        r.setLogCallback(dashcam::log::getCallback());
        r.setLumaTap(true);
        rec::RecorderTestHook::setSource(r, src.desc);
        auto o = segOpts(dir, 60);
        o.maxFps = 5;
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), tag + ": start (RecordFps 5, luma tap)");
        feedClockOnly(r, 3000);
        const std::string caps = recFramerate(r);
        float luma = 0;
        uint64_t lumaSeq = 0;
        const bool gotLuma = r.latestLuma(luma, lumaSeq);
        const uint64_t source = r.sourceFramesReceived();
        check(r.isRecording(), tag + ": recording healthy");
        check(r.stop(), tag + ": stop finalised");
        const uint64_t frames = r.framesReceived();
        bool ok = false;
        const int total = recordedFrames(dir, ok);
        check(caps == "5/1", tag + ": recording negotiated at the cap (framerate " + caps + ")");
        check(frames >= 12 && frames <= 19 && ok && (uint64_t)total == frames,
              tag + ": ~5 fps recorded, gapless (" + std::to_string(total) + "/" + std::to_string(frames) +
              " frames in 3 s)");
        check(source >= 70 && source <= 105, tag + ": camera still at 30 fps (" + std::to_string(source) +
              " source frames)");
        check(gotLuma && lumaSeq >= 4 && lumaSeq <= 8, tag + ": luma samples ~2/s (" +
              std::to_string(lumaSeq) + " in 3 s)");
        fs::remove_all(dir);
    }
}

// ─── A10: splitNow (clock corrected → new segment) ────────────────────────────

static void testSplitNow() {
    std::cout << "\n--- A10: splitNow ---\n";
    const std::string dir = makeTempDir("split");
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    check(!r.splitNow(), "no split before a session");
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)), "start (60 s segments)");
    feedClockOnly(r, 2000);
    check(r.splitNow(), "split requested mid-segment");
    feedClockOnly(r, 2000);
    check(r.isRecording(), "healthy after the split");
    check(r.stop(), "stop finalised");
    const uint64_t frames = r.framesReceived();
    const auto segs = listSegments(dir);
    int total = 0;
    bool ok = true;
    for (const auto& [seq, files] : segs) {
        const MkvInfo mi = probeMkv(files.first);
        total += mi.frames;
        ok = ok && mi.ok && mi.firstPts >= 0 && mi.firstPts < 100'000'000LL && readAss(files.second).exists;
    }
    check(segs.size() == 2, std::to_string(segs.size()) + " segments (split + remainder)");
    check(ok, "both segments start at t~0 with their own sidecar");
    check((uint64_t)total == frames, "gapless across the split (" + std::to_string(total) + "/" +
          std::to_string(frames) + " frames)");
    fs::remove_all(dir);
}

// ─── A11: stop()'s deadline covers a worker stuck in sidecar I/O ─────────────

// Child half (record_test --hang-child <dir> [full-stderr]): the segment's .ass
// path is a FIFO nobody reads, so the worker blocks forever opening it — the
// stand-in for a wedged filesystem.  stop() must still _exit(3) at
// eosTimeoutMs + 3 s — also when stderr is a full pipe nobody drains (a
// stalled log collector) while the recorder logs through liblog's real
// fallback (no init(): synchronous stderr, so any log call there blocks).
static int hangChild(const std::string& dir, bool fullStderr) {
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    auto o = segOpts(dir, 2);                             // roll over after 2 s
    o.eosTimeoutMs       = 100;                           // deadline = 3.1 s
    o.exitOnTeardownHang = true;                          // the production behaviour
    if (!r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o)) return 10;
    // Wait for the first segment, then plant FIFOs for the NEXT one's sidecar
    // (planting them earlier would bump the session's starting SEQ past them).
    for (int i = 0; i < 50 && r.currentFile().empty(); ++i) sleepMs(100);
    const auto seq = rec::parseSegmentName(fs::path(r.currentFile()).filename().string(), "dashcam");
    if (!seq) return 11;
    const time_t now = std::time(nullptr);
    for (time_t t = now - 1; t <= now + 10; ++t) {
        std::string ass = rec::segmentFileName("dashcam", *seq + 1, t);
        ass.replace(ass.size() - 4, 4, ".ass");
        ::mkfifo((dir + "/" + ass).c_str(), 0644);
    }
    sleepMs(3500);                                        // rollover: worker now stuck in open()
    if (fullStderr) {
        int p[2];
        if (::pipe(p) != 0) return 12;
        ::fcntl(p[1], F_SETFL, O_NONBLOCK);
        static const char junk[4096] = {};
        while (::write(p[1], junk, sizeof junk) > 0) {}   // fill it...
        ::fcntl(p[1], F_SETFL, 0);                        // ...and make writes block again
        ::dup2(p[1], STDERR_FILENO);                      // p[0] stays open, never read
    }
    r.stop();                                             // must not return
    return 0;
}

static void runHangChild(bool fullStderr) {
    const std::string tag = fullStderr ? "full stderr pipe: " : "";
    const std::string dir = makeTempDir("hang");
    const auto t0 = std::chrono::steady_clock::now();
    const pid_t pid = ::fork();
    if (pid == 0) {
        ::execl("/proc/self/exe", "record_test", "--hang-child", dir.c_str(),
                fullStderr ? "full-stderr" : "plain", static_cast<char*>(nullptr));
        ::_exit(127);
    }
    check(pid > 0, tag + "child started");
    int status = 0;
    bool exited = false;
    while (pid > 0 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(15)) {
        if (::waitpid(pid, &status, WNOHANG) == pid) { exited = true; break; }
        sleepMs(50);
    }
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (!exited && pid > 0) { ::kill(pid, SIGKILL); ::waitpid(pid, &status, 0); }
    check(exited, tag + "stop() did not hang past its deadline (" + std::to_string(secs) + " s)");
    check(exited && WIFEXITED(status) && WEXITSTATUS(status) == 3,
          tag + "worker really was blocked and the guard exited with 3 (status " +
          std::to_string(exited && WIFEXITED(status) ? WEXITSTATUS(status) : -1) + ")");
    check(exited && secs < 11.0, tag + "exit within start + 3.5 s + the 3.1 s deadline (+ slack)");
    fs::remove_all(dir);
}

static void testStopDeadlineCoversWorker() {
    std::cout << "\n--- A11: stop() deadline covers a worker blocked on sidecar I/O ---\n";
    runHangChild(false);
    runHangChild(true);
}

// ─── A12: periodic disk sync (power-cut durability) ───────────────────────────

static void testDurableSync() {
    std::cout << "\n--- A12: periodic disk sync ---\n";
    for (uint32_t interval : {500u, 0u}) {
        const std::string tag = interval ? "sync 500 ms" : "sync off";
        const std::string dir = makeTempDir("sync");
        rec::SegmentedRecorder r;
        r.setLogCallback(dashcam::log::getCallback());
        rec::RecorderTestHook::setSource(r, kMjpegSrc);
        auto o = segOpts(dir, 2);                         // rotations mid-run
        o.syncIntervalMs = interval;
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), tag + ": start");
        feedClockOnly(r, 4500);
        const uint64_t syncs  = r.syncCount();
        const uint64_t synced = r.syncedFileCount();
        check(r.isRecording(), tag + ": healthy while syncing");
        check(r.stop(), tag + ": stop finalised");
        if (interval) {
            check(syncs >= 7 && syncs <= 10, tag + ": ~2 sync rounds per second (" + std::to_string(syncs) + " in 4.5 s)");
            // Each round syncs the open segment and its sidecar (the very first
            // round may run before either exists).
            check(synced >= 2 * syncs - 2, tag + ": segment + sidecar really synced each round (" +
                  std::to_string(synced) + " file syncs in " + std::to_string(syncs) + " rounds)");
        } else {
            check(syncs == 0 && synced == 0, tag + ": no sync rounds, no file syncs");
        }
        const uint64_t frames = r.framesReceived();
        int total = 0;
        bool ok = true;
        const auto segs = listSegments(dir);
        for (const auto& [seq, files] : segs) {
            const MkvInfo mi = probeMkv(files.first);
            total += mi.frames;
            ok = ok && mi.ok && readAss(files.second).exists;
        }
        check(segs.size() >= 2 && ok, tag + ": " + std::to_string(segs.size()) + " playable segments with sidecars");
        check((uint64_t)total == frames, tag + ": gapless (" + std::to_string(total) + "/" + std::to_string(frames) + ")");
        fs::remove_all(dir);
    }
}

// ─── A13: an abrupt exit at a low RecordFps loses only the newest frame ───────

// Child half (record_test --abrupt-child <dir>): records at RecordFps 2 and dies
// without stop() just after a frame arrives — what a power cut does to the
// process (the page cache survives; making it durable is the sync thread's
// part).  Writes the frames it had received to <dir>/received.txt first.
static int abruptChild(const std::string& dir) {
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    auto o = segOpts(dir, 60);
    o.maxFps = 2;                                         // videorate drop-only, as RecordFps=2
    if (!r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o)) return 10;
    for (int i = 0; i < 200 && r.framesReceived() < 8; ++i) sleepMs(25);
    const uint64_t before = r.framesReceived();
    for (int i = 0; i < 200 && r.framesReceived() == before; ++i) sleepMs(5);
    sleepMs(150);                                         // well inside the 500 ms frame gap
    const uint64_t n = r.framesReceived();
    if (FILE* f = std::fopen((dir + "/received.txt").c_str(), "w")) {
        std::fprintf(f, "%llu\n", static_cast<unsigned long long>(n));
        std::fclose(f);
    }
    ::_exit(0);                                           // no stop(), no finalise
}

static void testAbruptExitLowRate() {
    std::cout << "\n--- A13: abrupt exit at RecordFps 2 ---\n";
    const std::string dir = makeTempDir("abrupt");
    const pid_t pid = ::fork();
    if (pid == 0) {
        ::execl("/proc/self/exe", "record_test", "--abrupt-child", dir.c_str(), static_cast<char*>(nullptr));
        ::_exit(127);
    }
    int status = 0;
    const auto t0 = std::chrono::steady_clock::now();
    bool exited = false;
    while (pid > 0 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(20)) {
        if (::waitpid(pid, &status, WNOHANG) == pid) { exited = true; break; }
        sleepMs(50);
    }
    if (!exited && pid > 0) { ::kill(pid, SIGKILL); ::waitpid(pid, &status, 0); }
    check(exited && WIFEXITED(status) && WEXITSTATUS(status) == 0, "child recorded and exited abruptly");
    long long received = -1;
    if (std::ifstream in{dir + "/received.txt"}) in >> received;
    const auto segs = listSegments(dir);
    const MkvInfo mi = segs.empty() ? MkvInfo{} : probeMkv(segs.begin()->second.first);
    check(received >= 8 && segs.size() == 1, "one unfinalised segment, " + std::to_string(received) +
          " frames received");
    // Only the newest frame may be missing: splitmuxsink holds it until the
    // next keyframe.  Nothing may sit in a user-space file buffer.
    check(received > 0 && mi.frames >= received - 1 && mi.frames <= received,
          "file holds " + std::to_string(mi.frames) + " of " + std::to_string(received) +
          " frames (at most the newest one lost)");
    fs::remove_all(dir);
}

// ─── A14: a stuck disk sync is reported while stuck, then ends the session ────

// syncIntervalMs 500 → warning after max(3 s, 1 s) = 3 s; the session goes
// unhealthy at max(stallMs, 2 x 3 s).  blockSidecar: the worker's sidecar
// writes hang on the same disk too — the watchdog must not depend on them.
static void syncStallCase(uint32_t stallMs, double expectFailAt, bool blockSidecar = false) {
    const std::string tag = "stall limit " + std::to_string(stallMs) + " ms" +
                            (blockSidecar ? " + sidecar I/O blocked" : "");
    const std::string dir = makeTempDir("stall");
    std::mutex               lm;
    std::vector<std::string> logs;
    auto base = dashcam::log::getCallback();
    auto logged = [&](const char* what) {
        std::lock_guard<std::mutex> lk(lm);
        for (const auto& l : logs) if (l.find(what) != std::string::npos) return true;
        return false;
    };
    rec::SegmentedRecorder r;
    r.setLogCallback([&](dashcam::log::LogLevel lv, const std::string& m) {
        { std::lock_guard<std::mutex> lk(lm); logs.push_back(m); }
        if (base) base(lv, m);
    });
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    std::atomic<bool> stall{false}, release{false};
    rec::RecorderTestHook::setSyncHook(r, [&] {
        if (!stall.load()) return;
        while (!release.load()) sleepMs(10);               // a disk that stopped answering
    });
    if (blockSidecar)
        rec::RecorderTestHook::setWorkerHook(r, [&] {
            if (!stall.load()) return;
            while (!release.load()) sleepMs(10);
        });
    auto o = segOpts(dir, 60);
    o.syncIntervalMs     = 500;
    o.syncStallTimeoutMs = stallMs;
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), tag + ": start");
    feedClockOnly(r, 1500);
    const uint64_t roundsBefore = r.syncCount();
    stall.store(true);
    const auto t0 = std::chrono::steady_clock::now();
    auto secs = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };
    double warnAt = -1, failAt = -1;
    while (secs() < expectFailAt + 4 && failAt < 0) {
        feedClockOnly(r, 100);
        if (warnAt < 0 && logged("disk sync not responding")) warnAt = secs();
        if (!r.isRecording()) failAt = secs();
    }
    check(roundsBefore >= 2 && r.syncCount() <= roundsBefore + 1, tag + ": no sync round completes while stuck");
    check(warnAt >= 2.5 && warnAt < 4.5, tag + ": warned while still stuck (" + std::to_string(warnAt) + " s)");
    check(failAt >= expectFailAt - 0.5 && failAt < expectFailAt + 1.5 && failAt > warnAt,
          tag + ": unhealthy after the warning, at ~" + std::to_string(int(expectFailAt)) + " s (" +
          std::to_string(failAt) + " s)");
    check(r.lastError().find("disk sync stuck") != std::string::npos, tag + ": reason: " + r.lastError());
    release.store(true);                                  // the disk answers again
    check(r.stop(), tag + ": stop() finalises once the disk answers");
    const auto segs = listSegments(dir);
    check(segs.size() == 1 && probeMkv(segs.begin()->second.first).ok, tag + ": segment playable");
    fs::remove_all(dir);
}

static void testSyncStall() {
    std::cout << "\n--- A14: stuck disk sync ---\n";
    syncStallCase(7000, 7.0);                             // the configured limit
    syncStallCase(1000, 6.0);                             // below 2 x warning: floored, warning first
    syncStallCase(1000, 6.0, true);                       // worker blocked on the same disk
}

// ─── A15: compressed preview tap (ES3C28P display preview) ───────────────────

static uint64_t monoUs() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

// Log lines kept for assertions, still forwarded to the real logger.
struct LogCapture {
    std::mutex                m;
    std::vector<std::string>  lines;
    dashcam::log::LogCallback base = dashcam::log::getCallback();
    dashcam::log::LogCallback callback() {
        return [this](dashcam::log::LogLevel lv, const std::string& s) {
            { std::lock_guard<std::mutex> lk(m); lines.push_back(s); }
            if (base) base(lv, s);
        };
    }
    int count(const std::string& what) {
        std::lock_guard<std::mutex> lk(m);
        int n = 0;
        for (const auto& l : lines) n += l.find(what) != std::string::npos;
        return n;
    }
    std::string first(const std::string& what) {
        std::lock_guard<std::mutex> lk(m);
        for (const auto& l : lines) if (l.find(what) != std::string::npos) return l;
        return "";
    }
};

// Everything a preview consumer saw, plus breaches of the callback contract.
struct PreviewLog {
    struct Frame {
        uint64_t seq = 0, captureUs = 0, arrivalUs = 0, hash = 0;
        uint32_t width = 0, height = 0, fourcc = 0;
        bool     jpeg = false;                ///< SOI first and EOI last.
    };
    std::mutex         m;
    std::vector<Frame> frames;
    std::atomic<int>   delayMs{0};            ///< > 0: a slow consumer.
    std::atomic<int>   throwAt{0};            ///< > 0: throw from this sequence number on.
    std::atomic<int>   inCallback{0};
    std::atomic<int>   afterStop{0};          ///< Calls that began after stop() returned.
    std::atomic<bool>  stopped{false};

    rec::PreviewFrameCallback callback() {
        return [this](const rec::PreviewFrame& f) {
            if (stopped.load()) ++afterStop;
            ++inCallback;
            Frame fr;
            fr.seq       = f.sequence;
            fr.captureUs = f.captureMonotonicUs;
            fr.arrivalUs = monoUs();
            fr.width     = f.width;
            fr.height    = f.height;
            fr.fourcc    = f.v4l2PixFmt;
            fr.jpeg      = f.data && f.size >= 4 && f.data[0] == 0xFF && f.data[1] == 0xD8 &&
                           f.data[f.size - 2] == 0xFF && f.data[f.size - 1] == 0xD9;
            fr.hash      = f.data ? fnv1a(f.data, f.size) : 0;
            { std::lock_guard<std::mutex> lk(m); frames.push_back(fr); }
            if (const int ms = delayMs.load()) sleepMs(ms);
            --inCallback;
            const int t = throwAt.load();
            if (t > 0 && f.sequence >= uint64_t(t)) throw std::runtime_error("synthetic consumer failure");
        };
    }
    std::vector<Frame> snapshot() { std::lock_guard<std::mutex> lk(m); return frames; }
    size_t count() { std::lock_guard<std::mutex> lk(m); return frames.size(); }
};

// Per-frame and rate checks over frames[from..]: JPEG + format + 320x240,
// sequence = index + 1, capture times increasing and never after arrival.
struct PreviewCheck {
    bool     jpeg = true, dims = true, seq = true, mono = true, lag = true;
    double   fps = 0;                         ///< Over the capture-time span.
    int64_t  minGapUs = INT64_MAX;
    uint64_t maxLagUs = 0;                    ///< Worst arrival − capture.
};

static PreviewCheck checkFrames(const std::vector<PreviewLog::Frame>& v, size_t from = 0) {
    PreviewCheck c;
    for (size_t i = from; i < v.size(); ++i) {
        const auto& f = v[i];
        c.jpeg = c.jpeg && f.jpeg && f.fourcc == V4L2_PIX_FMT_MJPEG;
        c.dims = c.dims && f.width == 320 && f.height == 240;
        c.seq  = c.seq && f.seq == i + 1;
        c.lag  = c.lag && f.captureUs > 0 && f.captureUs <= f.arrivalUs;
        if (f.captureUs <= f.arrivalUs) c.maxLagUs = std::max(c.maxLagUs, f.arrivalUs - f.captureUs);
        if (i > from) {
            const int64_t gap = int64_t(f.captureUs) - int64_t(v[i - 1].captureUs);
            c.mono     = c.mono && gap > 0;
            c.minGapUs = std::min(c.minGapUs, gap);
        }
    }
    if (v.size() >= from + 2 && v.back().captureUs > v[from].captureUs)
        c.fps = double(v.size() - from - 1) * 1e6 / double(v.back().captureUs - v[from].captureUs);
    return c;
}

static bool hasElement(rec::SegmentedRecorder& r, const char* name) {
    GstElement* e = gst_bin_get_by_name(GST_BIN(rec::RecorderTestHook::pipeline(r)), name);
    if (e) gst_object_unref(e);
    return e != nullptr;
}

// A15a: delivery contract at 10 fps from a 30 fps camera; the recording is the
// same with the tap as without it, and the tapped bytes are recorded bytes.
static void testPreviewDelivery() {
    std::cout << "\n--- A15a: preview tap delivers rate-limited JPEGs, recording unchanged ---\n";
    uint64_t framesOff = 0;
    size_t   segsOff   = 0;
    {
        const std::string dir = makeTempDir("prevoff");
        rec::SegmentedRecorder r;
        r.setLogCallback(dashcam::log::getCallback());
        rec::RecorderTestHook::setSource(r, kMjpegSrc);
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 2)), "tap off: start");
        check(!r.previewActive() && !hasElement(r, "srctee"), "tap off: no tee, no preview branch");
        feedClockOnly(r, 4000);
        check(r.stop(), "tap off: stop finalised");
        framesOff = r.framesReceived();
        bool ok = false;
        const int total = recordedFrames(dir, ok);
        segsOff = listSegments(dir).size();
        check(ok && (uint64_t)total == framesOff, "tap off: gapless (" + std::to_string(total) + "/" +
              std::to_string(framesOff) + ")");
        fs::remove_all(dir);
    }

    PreviewLog pl;
    LogCapture logs;
    const std::string dir = makeTempDir("prevon");
    rec::SegmentedRecorder r;
    r.setLogCallback(logs.callback());
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    r.setPreviewTap(pl.callback(), 10);
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 2)), "tap on: start");
    check(r.previewActive(), "tap on: preview branch active");
    check(logs.count(" ! tee name=srctee srctee. ! queue name=recq") == 1 &&
          logs.count(" srctee. ! queue name=previewq max-size-buffers=1 leaky=downstream "
                     "! appsink name=previewsink ") == 1 &&
          logs.count("videorate") == 0,
          "pipeline: tee after the source, preview* branch as specified (no caps-negotiating videorate)");
    feedClockOnly(r, 4000);
    check(r.isRecording() && r.lastError().empty(), "tap on: recording healthy");
    const bool clean = r.stop();
    pl.stopped.store(true);
    check(clean, "tap on: stop finalised");
    check(!r.previewActive(), "no preview branch after stop()");

    const auto v = pl.snapshot();
    const auto c = checkFrames(v);
    check(v.size() >= 30 && v.size() <= 44, std::to_string(v.size()) + " preview frames in 4 s at 10 fps");
    check(r.previewFramesTapped() == v.size(), "previewFramesTapped() == frames delivered (kept after stop)");
    check(c.jpeg, "every frame is one complete JPEG (SOI..EOI), fourcc MJPG");
    check(c.dims, "width/height 320x240 from the negotiated caps");
    check(c.seq, "sequence 1, 2, 3 ... without gaps");
    check(c.mono, "capture timestamps strictly increasing");
    check(c.lag && c.maxLagUs < 250000, "capture time <= arrival, lag < 250 ms (max " +
          std::to_string(c.maxLagUs / 1000) + " ms)");
    check(c.fps > 8.0 && c.fps <= 10.5, "rate " + std::to_string(c.fps) + " fps <= maxFps 10");
    check(c.minGapUs >= 74000, "no burst: min capture gap " + std::to_string(c.minGapUs / 1000) + " ms");
    check(logs.count("capture times from buffer PTS (pipeline clock is CLOCK_MONOTONIC)") == 1,
          "timestamps from PTS + base time (system clock verified monotonic)");

    const uint64_t framesOn = r.framesReceived();
    std::set<uint64_t> recorded;
    bool ok = false;
    const int total = recordedFrames(dir, ok, &recorded);
    check(ok && (uint64_t)total == framesOn, "tap on: gapless (" + std::to_string(total) + "/" +
          std::to_string(framesOn) + ")");
    check(framesOn * 100 >= framesOff * 85 && framesOn * 100 <= framesOff * 115,
          "recorded frame count unaffected by the tap (" + std::to_string(framesOn) + " vs " +
          std::to_string(framesOff) + " without)");
    const size_t segsOn = listSegments(dir).size();
    check(segsOn + 1 >= segsOff && segsOn <= segsOff + 1, "segment output unaffected (" +
          std::to_string(segsOn) + " vs " + std::to_string(segsOff) + " segments)");
    size_t inFile = 0;
    for (const auto& f : v) inFile += recorded.count(f.hash);
    check(!v.empty() && inFile == v.size(), "tapped bytes are recorded bytes (" + std::to_string(inFile) +
          "/" + std::to_string(v.size()) + " frames found in the segments)");
    check(pl.afterStop.load() == 0, "no callback after stop()");
    fs::remove_all(dir);
}

// A15b: a RecordFps cap below the preview rate does not cap the preview, and
// sourceFramesReceived() still counts camera frames.  (With the luma tap as
// well: A15f.)
static void testPreviewWithRecordFps() {
    std::cout << "\n--- A15b: RecordFps 5 does not cap the preview (15 fps) ---\n";
    PreviewLog pl;
    LogCapture logs;
    const std::string dir = makeTempDir("prevcap");
    rec::SegmentedRecorder r;
    r.setLogCallback(logs.callback());
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    r.setPreviewTap(pl.callback(), 15);
    auto o = segOpts(dir, 60);
    o.maxFps = 5;
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), "start");
    const std::string line = logs.first("segmented recording pipeline: ");
    const size_t tee = line.find("tee name=srctee"), rate = line.find("videorate name=recrate");
    check(tee != std::string::npos && rate != std::string::npos && tee < rate,
          "preview tee splits off BEFORE the recrate cap");
    feedClockOnly(r, 3000);
    const uint64_t source = r.sourceFramesReceived();
    check(r.isRecording(), "recording healthy");
    check(r.stop(), "stop finalised");

    const auto v = pl.snapshot();
    const auto c = checkFrames(v);
    check(c.fps > 12.0 && c.fps <= 15.75, "preview at " + std::to_string(c.fps) +
          " fps despite RecordFps 5 (" + std::to_string(v.size()) + " frames)");
    check(c.jpeg && c.dims && c.seq && c.mono && c.lag, "preview frames well-formed");
    check(c.minGapUs >= 49000, "no burst: min capture gap " + std::to_string(c.minGapUs / 1000) + " ms");
    check(source >= 70 && source <= 105, "sourceFramesReceived() counts camera frames (" +
          std::to_string(source) + " in 3 s)");
    const uint64_t recorded = r.framesReceived();
    check(recorded >= 12 && recorded <= 19, "recording capped at 5 fps (" + std::to_string(recorded) +
          " frames)");
    bool ok = false;
    const int total = recordedFrames(dir, ok);
    check(ok && (uint64_t)total == recorded, "gapless (" + std::to_string(total) + "/" +
          std::to_string(recorded) + ")");
    fs::remove_all(dir);
}

// A15b2: the luma tap and the preview tap together, each at its own rate.
static void testPreviewWithLuma() {
    std::cout << "\n--- A15b2: luma tap + preview tap together ---\n";
    PreviewLog pl;
    const std::string dir = makeTempDir("prevluma");
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    r.setLumaTap(true);
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    r.setPreviewTap(pl.callback(), 10);
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)), "start");
    check(hasElement(r, "lumasink") && hasElement(r, "previewsink"), "both branches built");
    feedClockOnly(r, 3000);
    float luma = 0;
    uint64_t lumaSeq = 0;
    const bool gotLuma = r.latestLuma(luma, lumaSeq);
    check(r.isRecording(), "recording healthy with both taps");
    check(r.stop(), "stop finalised");
    check(gotLuma && lumaSeq >= 4 && lumaSeq <= 8, "luma tap ~2 samples/s (" + std::to_string(lumaSeq) + ")");
    const auto v = pl.snapshot();
    const auto c = checkFrames(v);
    check(c.fps > 8.0 && c.fps <= 10.5 && c.jpeg && c.seq && c.mono, "preview at " +
          std::to_string(c.fps) + " fps, well-formed (" + std::to_string(v.size()) + " frames)");
    const uint64_t frames = r.framesReceived();
    std::set<uint64_t> recorded;
    bool ok = false;
    const int total = recordedFrames(dir, ok, &recorded);
    size_t inFile = 0;
    for (const auto& f : v) inFile += recorded.count(f.hash);
    check(frames >= 80 && ok && (uint64_t)total == frames, "every camera frame recorded (" +
          std::to_string(total) + "/" + std::to_string(frames) + ")");
    check(!v.empty() && inFile == v.size(), "tapped bytes are recorded bytes (" +
          std::to_string(inFile) + "/" + std::to_string(v.size()) + ")");
    fs::remove_all(dir);
}

// A15c: a callback that blocks for 300 ms costs the preview frames, never the
// recording; when it speeds up again the rate stays capped (no videorate
// catch-up burst); stop() waits for a call in progress and none starts after.
static void testPreviewSlowConsumer() {
    std::cout << "\n--- A15c: slow preview consumer, burst guard, no callback after stop() ---\n";
    PreviewLog pl;
    const std::string dir = makeTempDir("prevslow");
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    r.setPreviewTap(pl.callback(), 15);
    auto o = segOpts(dir, 60);
    o.stallTimeoutMs = 1000;                              // a blocked source trips this fast
    pl.delayMs.store(300);
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), "start");
    feedClockOnly(r, 2500);
    const uint64_t slowFrames = r.framesReceived();
    const size_t   slowTaps   = pl.count();
    check(r.isRecording() && r.lastError().empty(), "recording healthy behind a 300 ms callback");
    check(slowFrames >= 60, "camera frames keep flowing (" + std::to_string(slowFrames) + " in 2.5 s)");
    check(slowTaps >= 5 && slowTaps <= 10, "slow consumer got ~1 frame / 300 ms (" +
          std::to_string(slowTaps) + ")");

    pl.delayMs.store(0);                                  // fast again
    const uint64_t fastFrom = monoUs() + 400000;          // past the call in flight
    feedClockOnly(r, 2500);
    const auto v = pl.snapshot();
    size_t from = v.size();
    for (size_t i = 0; i < v.size(); ++i) if (v[i].arrivalUs >= fastFrom) { from = i; break; }
    const auto c = checkFrames(v, from);
    check(v.size() - from >= 20, std::to_string(v.size() - from) + " frames in the fast phase");
    check(c.fps <= 15.75 && c.minGapUs >= 49000, "no catch-up burst after the slow phase (" +
          std::to_string(c.fps) + " fps, min gap " + std::to_string(c.minGapUs / 1000) + " ms)");

    pl.delayMs.store(300);                                // so stop() lands inside a call
    sleepMs(450);
    const int busy = pl.inCallback.load();
    const auto t0 = std::chrono::steady_clock::now();
    const bool clean = r.stop();
    const int busyAfter = pl.inCallback.load();
    pl.stopped.store(true);
    const double stopS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    sleepMs(500);
    check(clean && stopS < 2.0, "stop finalised in " + std::to_string(stopS) + " s");
    std::cout << "    callback in progress when stop() began: " << busy << "\n";
    check(busyAfter == 0, "stop() returned only after the call in progress returned");
    check(pl.afterStop.load() == 0, "no callback after stop() returned");
    const auto all = pl.snapshot();
    const auto ca = checkFrames(all);
    check(ca.seq && ca.mono && ca.jpeg, "sequence and timestamps consistent across all phases");
    bool ok = false;
    const uint64_t frames = r.framesReceived();
    const int total = recordedFrames(dir, ok);
    check(frames >= 140, "recording kept the camera rate (" + std::to_string(frames) + " in ~5.5 s)");
    check(ok && (uint64_t)total == frames, "gapless (" + std::to_string(total) + "/" +
          std::to_string(frames) + ")");
    fs::remove_all(dir);
}

// A15d: configuration — H.264 gets no branch (INFO), maxFps 0 / empty callback
// build none, and a change during a session waits for the next start().
static void testPreviewConfig() {
    std::cout << "\n--- A15d: preview tap configuration ---\n";
    {
        PreviewLog pl;
        LogCapture logs;
        const std::string dir = makeTempDir("prev264");
        rec::SegmentedRecorder r;
        r.setLogCallback(logs.callback());
        r.setPreviewTap(pl.callback(), 10);
        rec::RecorderTestHook::setSource(
            r, "videotestsrc is-live=true ! video/x-raw,width=320,height=240,framerate=30/1 "
               "! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30");
        check(r.start("unused", fmt320(V4L2_PIX_FMT_H264), segOpts(dir, 60)), "H.264: start");
        check(!r.previewActive() && !hasElement(r, "previewsink") && !hasElement(r, "srctee"),
              "H.264: no preview branch");
        check(logs.count("preview tap needs an MJPEG source") == 1, "H.264: logged once at INFO");
        sleepMs(1500);
        check(r.isRecording() && pl.count() == 0 && r.previewFramesTapped() == 0,
              "H.264: recording healthy, no preview frames");
        check(r.stop(), "H.264: stop finalised");
        fs::remove_all(dir);
    }
    for (int variant = 0; variant < 2; ++variant) {
        PreviewLog pl;
        const std::string tag = variant == 0 ? "maxFps 0" : "empty callback";
        const std::string dir = makeTempDir("prevnone");
        rec::SegmentedRecorder r;
        r.setLogCallback(dashcam::log::getCallback());
        r.setPreviewTap(variant == 0 ? pl.callback() : rec::PreviewFrameCallback{}, variant == 0 ? 0 : 10);
        rec::RecorderTestHook::setSource(r, kMjpegSrc);
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)), tag + ": start");
        check(!r.previewActive() && !hasElement(r, "srctee"), tag + ": no branch");
        sleepMs(500);
        check(r.stop() && pl.count() == 0, tag + ": recorded, nothing tapped");
        fs::remove_all(dir);
    }
    {
        PreviewLog a, b;
        LogCapture logs;
        const std::string dir = makeTempDir("prevswap");
        rec::SegmentedRecorder r;
        r.setLogCallback(logs.callback());
        rec::RecorderTestHook::setSource(r, kMjpegSrc);
        r.setPreviewTap(a.callback(), 10);
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)), "swap: start with tap A");
        sleepMs(700);
        r.setPreviewTap(b.callback(), 10);                // mid-session: stored, not applied
        check(logs.count("takes effect at the next start()") == 1, "mid-session change logged at INFO");
        const size_t aAtSwap = a.count();
        sleepMs(1000);
        check(a.count() >= aAtSwap + 7 && b.count() == 0, "running session keeps tap A (A " +
              std::to_string(a.count()) + ", B " + std::to_string(b.count()) + ")");
        check(r.stop(), "swap: stop");
        const size_t aFinal = a.count();
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)), "swap: restart");
        sleepMs(1000);
        check(b.count() >= 7 && a.count() == aFinal, "next session delivers to tap B (B " +
              std::to_string(b.count()) + ")");
        const auto bf = b.snapshot();
        check(!bf.empty() && bf.front().seq == 1, "sequence restarts at 1 in the new session");
        check(r.stop(), "swap: stop again");
        fs::remove_all(dir);
    }
}

// Returns a fatal flow from inside the preview branch (unref: HANDLED owns it).
static GstPadProbeReturn failFlow(GstPad*, GstPadProbeInfo* info, gpointer) {
    if (GstBuffer* b = GST_PAD_PROBE_INFO_BUFFER(info)) gst_buffer_unref(b);
    GST_PAD_PROBE_INFO_FLOW_RETURN(info) = GST_FLOW_ERROR;
    return GST_PAD_PROBE_HANDLED;
}

// A15e: a preview failure stops the preview and nothing else — a fatal flow in
// the branch (which a leaky queue alone would hand back up the tee to the
// camera), a bus ERROR from a preview* element, and a throwing callback.  In
// every case the recording stays healthy and gapless and stop() finalises
// promptly (the dead branch still reaches EOS).
static void testPreviewFailure() {
    std::cout << "\n--- A15e: preview failures stay out of the recording ---\n";
    for (int mode = 0; mode < 3; ++mode) {
        const char* tags[] = {"fatal flow in branch", "bus ERROR from previewsink", "throwing callback"};
        const std::string tag = tags[mode];
        PreviewLog pl;
        LogCapture logs;
        const std::string dir = makeTempDir("prevfail");
        rec::SegmentedRecorder r;
        r.setLogCallback(logs.callback());
        rec::RecorderTestHook::setSource(r, kMjpegSrc);
        if (mode == 2) pl.throwAt.store(4);
        r.setPreviewTap(pl.callback(), 15);
        auto o = segOpts(dir, 60);
        o.stallTimeoutMs = 1000;
        check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), tag + ": start");
        feedClockOnly(r, 1000);
        GstElement* pipe = rec::RecorderTestHook::pipeline(r);
        if (mode == 0) {
            GstElement* pq = gst_bin_get_by_name(GST_BIN(pipe), "previewq");
            GstPad* pad = gst_element_get_static_pad(pq, "src");
            gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, failFlow, nullptr, nullptr);
            gst_object_unref(pad);
            gst_object_unref(pq);
        } else if (mode == 1) {
            GstElement* ps = gst_bin_get_by_name(GST_BIN(pipe), "previewsink");
            GError* e = g_error_new_literal(GST_STREAM_ERROR, GST_STREAM_ERROR_FAILED,
                                            "synthetic preview failure");
            gst_element_post_message(ps, gst_message_new_error(GST_OBJECT(ps), e, "record_test"));
            g_error_free(e);
            gst_object_unref(ps);
        }
        sleepMs(500);                                     // the failure lands
        const uint64_t framesMid = r.framesReceived();
        const uint64_t tapsMid   = r.previewFramesTapped();
        feedClockOnly(r, 1500);
        const uint64_t framesEnd = r.framesReceived();
        check(r.isRecording() && r.lastError().empty(), tag + ": recording healthy (" +
              (r.lastError().empty() ? std::string("no error") : r.lastError()) + ")");
        check(framesEnd >= framesMid + 35, tag + ": camera frames keep flowing (" +
              std::to_string(framesEnd - framesMid) + " in 1.5 s)");
        check(!r.previewActive() && r.previewFramesTapped() == tapsMid && tapsMid > 0,
              tag + ": preview stopped after " + std::to_string(tapsMid) + " frames");
        check(logs.count("preview tap error (") >= 1 && logs.count("preview stopped") >= 1,
              tag + ": WARN \"" + logs.first("preview tap error (") + "\"");
        check(logs.count("recording unhealthy") == 0, tag + ": recording never marked unhealthy");
        const auto t0 = std::chrono::steady_clock::now();
        const bool clean = r.stop();
        const double stopS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        check(clean && stopS < 2.0, tag + ": stop finalised in " + std::to_string(stopS) + " s");
        bool ok = false;
        const uint64_t frames = r.framesReceived();
        const int total = recordedFrames(dir, ok);
        check(ok && (uint64_t)total == frames, tag + ": gapless (" + std::to_string(total) + "/" +
              std::to_string(frames) + ")");
        fs::remove_all(dir);
    }
}

// A15f: the recorded rate is the camera's — or RecordFps — whatever the taps;
// the preview and luma taps keep their own rates.  Neither tap branch may take
// part in caps negotiation: a videorate max-rate in one pulled the recording
// (behind recrate) or, with a negotiable source, the camera itself down to the
// tap's rate.  The negotiable source runs uncapped only: recrate itself would
// negotiate it down to RecordFps (the recording's own element, not a tap's —
// the real v4l2src caps pin the rate, as the fixed-caps rows do).
static void testPreviewRateMatrix() {
    std::cout << "\n--- A15f: recorded rate with every mix of RecordFps, luma tap and source caps ---\n";
    for (const RateSource& src : rateSources(true)) {
        const std::string kind = src.tag;
        if (kind == "jpegenc caps") continue;                         // A15a/b/b2 cover it
        for (const uint32_t cap : {0u, 5u}) {
            if (cap && kind == "negotiable rate") continue;
            for (const bool luma : {false, true}) {
                const std::string tag = std::string(src.tag) + (cap ? ", RecordFps 5" : ", no cap") +
                                        (luma ? ", luma" : "");
                const double want = cap ? 5.0 : 30.0;
                PreviewLog pl;
                const std::string dir = makeTempDir("prevmatrix");
                rec::SegmentedRecorder r;
                r.setLogCallback(dashcam::log::getCallback());
                r.setLumaTap(luma);
                rec::RecorderTestHook::setSource(r, src.desc);
                r.setPreviewTap(pl.callback(), 15);
                auto o = segOpts(dir, 60);
                o.maxFps = cap;
                check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), o), tag + ": start");
                const Rates rt = measureRates(r, 2000);
                const std::string caps = recFramerate(r);
                check(r.isRecording() && r.previewActive(), tag + ": recording healthy, preview active");
                check(r.stop(), tag + ": stop finalised");
                const uint64_t frames = r.framesReceived();
                bool ok = false;
                const int total = recordedFrames(dir, ok);
                const std::string wantCaps = cap ? "5/1" : "30/1";
                check(caps == wantCaps && rt.rec >= want * 0.8 && rt.rec <= want * 1.2,
                      tag + ": recording at " + fmtFps(rt.rec) + " fps, caps " + caps + " (want " + wantCaps + ")");
                check(rt.source >= 27.0 && rt.source <= 33.0, tag + ": camera at " + fmtFps(rt.source) + " fps");
                check(rt.preview >= 12.0 && rt.preview <= 16.0, tag + ": preview at " + fmtFps(rt.preview) + " fps");
                if (luma)
                    check(rt.luma >= 1.0 && rt.luma <= 3.0, tag + ": luma at " + fmtFps(rt.luma) + " samples/s");
                check(ok && (uint64_t)total == frames, tag + ": gapless (" + std::to_string(total) + "/" +
                      std::to_string(frames) + ")");
                fs::remove_all(dir);
            }
        }
    }
}

// Fails the next buffer leaving previewq and closes the preview gate at that
// very moment, as a stop() beginning right then does: no later buffer can
// notice the failure before the EOS reaches the guard.
struct FailBeforeStop {
    rec::SegmentedRecorder* r = nullptr;
    std::atomic<bool>       armed{false}, fired{false};
};
static GstPadProbeReturn failAndCloseGate(GstPad*, GstPadProbeInfo* info, gpointer user) {
    auto* s = static_cast<FailBeforeStop*>(user);
    if (!s->armed.load() || s->fired.exchange(true)) return GST_PAD_PROBE_OK;
    rec::RecorderTestHook::closePreviewGate(*s->r);
    if (GstBuffer* b = GST_PAD_PROBE_INFO_BUFFER(info)) gst_buffer_unref(b);
    GST_PAD_PROBE_INFO_FLOW_RETURN(info) = GST_FLOW_ERROR;
    return GST_PAD_PROBE_HANDLED;
}

// A15g: a branch failure that only the EOS can notice (it landed just before
// stop() closed the gate).  The queue would refuse that EOS, so the guard must
// check it like a buffer and send the sink its EOS directly; otherwise the
// pipeline-wide EOS never comes and stop() waits out eosTimeoutMs.
static void testPreviewFailureAtStop() {
    std::cout << "\n--- A15g: preview failure just before stop() ---\n";
    PreviewLog pl;
    LogCapture logs;
    const std::string dir = makeTempDir("prevstopfail");
    rec::SegmentedRecorder r;
    r.setLogCallback(logs.callback());
    rec::RecorderTestHook::setSource(r, kMjpegSrc);
    r.setPreviewTap(pl.callback(), 15);
    check(r.start("unused", fmt320(V4L2_PIX_FMT_MJPEG), segOpts(dir, 60)), "start");
    feedClockOnly(r, 1000);
    FailBeforeStop fb;
    fb.r = &r;
    GstElement* pq  = gst_bin_get_by_name(GST_BIN(rec::RecorderTestHook::pipeline(r)), "previewq");
    GstPad*     pad = gst_element_get_static_pad(pq, "src");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, failAndCloseGate, &fb, nullptr);
    gst_object_unref(pad);
    gst_object_unref(pq);
    fb.armed.store(true);
    for (int i = 0; i < 200 && !fb.fired.load(); ++i) sleepMs(5);
    check(fb.fired.load(), "branch failed with the gate closed");
    const bool activeBefore = r.previewActive();
    const auto t0 = std::chrono::steady_clock::now();
    const bool clean = r.stop();
    const double stopS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    check(activeBefore, "failure not yet noticed before stop() (no buffer checked it)");
    check(clean && stopS < 2.0, "stop finalised in " + std::to_string(stopS) + " s (eosTimeoutMs 4 s)");
    check(logs.count("segment finalise timed out") == 0, "the pipeline-wide EOS arrived");
    check(logs.count("preview tap error (previewq)") >= 1, "WARN \"" + logs.first("preview tap error (") + "\"");
    bool ok = false;
    const uint64_t frames = r.framesReceived();
    const int total = recordedFrames(dir, ok);
    check(frames >= 25 && ok && (uint64_t)total == frames, "gapless (" + std::to_string(total) + "/" +
          std::to_string(frames) + ")");
    fs::remove_all(dir);
}

static void testPreviewTap() {
    testPreviewDelivery();
    testPreviewWithRecordFps();
    testPreviewWithLuma();
    testPreviewSlowConsumer();
    testPreviewConfig();
    testPreviewFailure();
    testPreviewRateMatrix();
    testPreviewFailureAtStop();
}

static void runPartA() {
    std::cout << "=== Part A: hardware-free ===\n";
    testGoldenSingleFile();
    testNaming();
    testSegmented("A3: segmented MJPEG (2 s segments, 7 s)", kMjpegSrc, V4L2_PIX_FMT_MJPEG);
    testSegmented("A4: segmented H.264 (GOP 1.5 s, 2 s segments, 7 s)",
                  "videotestsrc is-live=true pattern=ball ! video/x-raw,width=320,height=240,"
                  "framerate=30/1 ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=45 "
                  "! video/x-h264,profile=baseline",
                  V4L2_PIX_FMT_H264);
    testEosBeforeFirstBuffer();
    testStall();
    testSourceError();
    testRetention();
    testLumaTap();
    testLumaUnderRecordFps();
    testSplitNow();
    testStopDeadlineCoversWorker();
    testDurableSync();
    testAbruptExitLowRate();
    testSyncStall();
    testPreviewTap();
}

// ─── Part B6: live segmented recording on the real camera ─────────────────────

static void testLiveSegmented(const std::string& device, const rec::RecordingFormat& rf) {
    std::cout << "\n--- Test 9: live segmented recording on " << device << " (4 s segments, 10 s) ---\n";
    const std::string dir = makeTempDir("live");
    rec::SegmentedRecorder r;
    r.setLogCallback(dashcam::log::getCallback());
    r.setOverlayConfig(dashcam::config::OverlayConfig{});
    rec::SegmentOptions o;
    o.dir = dir;
    o.segmentSec = 4;
    o.exitOnTeardownHang = false;
    const bool started = r.start(device, rf, o);
    check(started, "live start OK");
    if (!started) { std::cout << "    " << r.lastError() << "\n"; fs::remove_all(dir); return; }
    feedClockOnly(r, 10000);
    check(r.isRecording(), "live session healthy");
    check(r.stop(), "live stop finalised cleanly");
    const uint64_t frames = r.framesReceived();
    const auto segs = listSegments(dir);
    check(segs.size() >= 3, std::to_string(segs.size()) + " live segments (>= 3)");
    int total = 0;
    bool ok = true;
    for (const auto& [seq, files] : segs) {
        const MkvInfo mi = probeMkv(files.first);
        const AssInfo ai = readAss(files.second);
        total += mi.frames;
        ok = ok && mi.ok && mi.firstPts >= 0 && mi.firstPts < 100'000'000LL && ai.exists &&
             ai.firstStartNs >= 0 && ai.firstStartNs < 500'000'000LL;
        std::cout << "    seq " << seq << ": " << mi.frames << " frames, " << fs::file_size(files.first) / 1000
                  << " kB, " << ai.clockEvents << " sidecar samples\n";
    }
    check(ok, "every live segment starts at t~0 with a covering sidecar");
    check((uint64_t)total == frames, "live gapless: " + std::to_string(total) +
          " frames in files vs " + std::to_string(frames) + " captured");
    // Aperture-priority auto exposure lowers the UVC frame rate in dim light
    // (the UGREEN drops to ~18 fps indoors), so only require that frames flow.
    check(frames >= 10 * 10, "live frames flowing (" + std::to_string(frames / 10) + " fps average)");
    fs::remove_all(dir);
}

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    if (argc >= 3 && std::string(argv[1]) == "--hang-child")
        return hangChild(argv[2], argc >= 4 && std::string(argv[3]) == "full-stderr");
    if (argc >= 3 && std::string(argv[1]) == "--abrupt-child") return abruptChild(argv[2]);
    const int seconds = argc > 1 ? std::stoi(argv[1]) : 10;

    auto log = dashcam::log::getCallback();

    runPartA();

    // ── Part B: find a USB camera with a compressed format ───────────────────
    std::cout << "\n=== Part B: USB camera ===\n";
    std::vector<cameraInfo> cams;
    getCameraList(cams, log);
    const cameraInfo* usb = nullptr;
    int fmtIdx = -1, rawIdx = -1;
    for (const auto& c : cams) {
        if (c.type != CAMERA_TYPE::USB) continue;
        for (size_t i = 0; i < c.videoFormats.size(); ++i) {
            const auto& f = c.videoFormats[i];
            const bool compressed = f.pixelFormat == V4L2_PIX_FMT_MJPEG ||
                                    f.pixelFormat == V4L2_PIX_FMT_H264;
            if (compressed && fmtIdx < 0 && f.width >= 1280 && f.width <= 1920 &&
                f.frameRate >= 24.0f && f.frameRate <= 31.0f)
                { usb = &c; fmtIdx = (int)i; }
            if (!compressed && rawIdx < 0) rawIdx = (int)i;
        }
        if (usb) break;
    }
    if (!usb || fmtIdx < 0) {
        std::cout << "no USB camera with a compressed <=1080p format\n";
        std::cout << "\n" << (g_fails == 0 ? "RESULT: PASS" : "RESULT: FAIL")
                  << " (" << g_fails << " failures) — Part B SKIPPED (no USB camera)\n";
        return g_fails == 0 ? 0 : 1;
    }
    const auto& f = usb->videoFormats[(size_t)fmtIdx];
    std::cout << "Using " << usb->address << " " << f.width << "x" << f.height
              << "@" << f.frameRate
              << (f.pixelFormat == V4L2_PIX_FMT_H264 ? " H264" : " MJPG") << "\n";

    dashcam::config::OverlayConfig ocfg;   // defaults: enabled, 5 Hz
    dashcam::record::Recorder rec;
    rec.setLogCallback(log);
    rec.setOverlayConfig(ocfg);

    // ── Test 1: raw formats are rejected ─────────────────────────────────────
    std::cout << "\n--- Test 1: raw format rejected ---\n";
    dashcam::record::RecordingFormat raw;
    raw.v4l2PixFmt = V4L2_PIX_FMT_YUYV;
    raw.width = 640; raw.height = 480; raw.fps = 30.0f;
    check(!rec.startRecording(usb->address, raw, "/tmp/should_not_exist.mkv"),
          "startRecording(YUYV) returns false");
    check(!fs::exists("/tmp/should_not_exist.mkv"), "no file created");

    // ── Test 2: compressed passthrough + ASS sidecar ─────────────────────────
    std::cout << "\n--- Test 2: passthrough recording + sidecar (" << seconds
              << " s) ---\n";
    const std::string mkv = "/tmp/record_test.mkv";
    const std::string ass = "/tmp/record_test.ass";
    fs::remove(mkv); fs::remove(ass);

    dashcam::record::RecordingFormat rf;
    rf.v4l2PixFmt = f.pixelFormat;
    rf.width = f.width; rf.height = f.height; rf.fps = f.frameRate;

    // Live-stream tap: fan the camera's own compressed frames to a viewer while
    // recording, validating the librecord tee/appsink → MediaStreamServer path.
    const bool mjpeg = f.pixelFormat == V4L2_PIX_FMT_MJPEG;
    net::MediaStreamServer streamSrv;
    net::StreamServerConfig sc;
    sc.port = 0;
    sc.wire = mjpeg ? net::StreamWire::MjpegHttp : net::StreamWire::RawTcp;
    bool streamStarted = streamSrv.start(sc, log);
    check(streamStarted, "stream server started");
    if (streamStarted)
        rec.setCompressedFrameCallback(
            [&](const uint8_t* d, size_t n, bool) { streamSrv.pushFrame(d, n); });

    check(rec.startRecording(usb->address, rf, mkv), "startRecording OK");

    // Connect a viewer once recording is under way.
    net::TcpSocket viewer;
    bool vconn = streamStarted && viewer.connect("127.0.0.1", streamSrv.port(), 1000, log);

    // Telemetry: live clock + a speed ramp so consecutive samples differ.
    for (int t = 0; t < seconds * 5 && rec.isRecording(); ++t) {
        dashcam::record::OverlayData od;
        od.timestampMs = epochMs();
        od.speedKmh    = 40.0f + 10.0f * std::sin(t * 0.2f);
        od.headingDeg  = static_cast<float>((t * 3) % 360);
        // Every per-source validity flag AND its own timestamp must be set, or
        // the renderer correctly draws dashes and this "fresh" case silently
        // tests nothing.  The flags default to false — fail-closed — so an
        // aggregate initialiser that omits them produces a blank overlay rather
        // than the live one the assertions below expect.
        od.speedValid          = true;
        od.accelValid          = true;
        od.positionValid       = true;
        od.headingValid        = true;
        od.speedTimestampMs    = od.timestampMs;
        od.accelTimestampMs    = od.timestampMs;
        od.positionTimestampMs = od.timestampMs;
        od.headingTimestampMs  = od.timestampMs;
        // ADAS telemetry banner (top-centre): lane position + fatigue state.
        // Both sources valid, so the banner shows real readings on both halves.
        od.adasValid       = true;
        od.laneValid       = true;
        od.driverValid     = true;
        od.laneCount       = 3;
        od.egoLaneIndex    = 1;
        od.laneOffset      = 0.25f;
        od.laneOffsetValid = true;
        od.fatigueScore    = 82.0f;
        od.fatigueLevel    = 2;      // WARN
        od.faceDetected    = true;
        rec.setOverlayData(od);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    check(rec.isRecording(), "session healthy for the whole run");

    // The viewer should have received live frames from the camera through the tap.
    std::string vs = drainSock(viewer, 1500);
    bool gotFrames = mjpeg
        ? (vs.find("Content-Type: image/jpeg") != std::string::npos &&
           vs.find(std::string("\xFF\xD8", 2)) != std::string::npos)   // JPEG SOI
        : (vconn && vs.size() > 1024);
    check(vconn, "stream viewer connected");
    check(gotFrames, "stream viewer received live compressed frames via the tap");
    check(streamStarted && streamSrv.clientCount() >= 1, "server still has the viewer");

    rec.stopRecording();
    streamSrv.stop();

    check(fs::exists(mkv) && fs::file_size(mkv) > 100 * 1024,
          "MKV exists and is non-trivial");
    check(fs::exists(ass), "ASS sidecar exists with the same basename");

    // ── Test 3: sidecar structure ────────────────────────────────────────────
    std::cout << "\n--- Test 3: sidecar structure ---\n";
    std::ifstream in(ass);
    std::string line;
    int styles = 0, events = 0, adasStyle = 0, adasEvents = 0;
    int adasReal = 0, adasFatDash = 0;
    bool playRes = false;
    while (std::getline(in, line)) {
        if (line.rfind("Style: ", 0) == 0)         ++styles;
        if (line.rfind("Style: ADAS,", 0) == 0)    ++adasStyle;
        if (line.rfind("Dialogue: ", 0) == 0)      ++events;
        if (line.find(",ADAS,,0,0,0,,") != std::string::npos) {
            ++adasEvents;
            // Both sources valid → real lane + fatigue readings, no dashes.
            if (line.find(",ADAS,,0,0,0,,LANE 2/3 +0.25   FAT 82 WARN") != std::string::npos)
                ++adasReal;
            if (line.find("FAT --") != std::string::npos) ++adasFatDash;
        }
        if (line == "PlayResX: " + std::to_string(f.width)) playRes = true;
    }
    check(styles == 6, "six styles (TL/TR/BL/BR + ADAS banner + DET block)");
    check(adasStyle == 1, "ADAS banner style present in header");
    check(adasEvents > 0, "ADAS telemetry banner rendered (" +
          std::to_string(adasEvents) + " events)");
    check(adasReal > 0, "ADAS banner shows real lane + fatigue readings when both valid");
    check(adasFatDash == 0, "no dashed FAT half while the driver source is valid");
    check(playRes, "PlayRes matches the video resolution");
    // 5 Hz nominal, 5 events per sample (4 corners + ADAS); generous startup slack.
    const int expectMin = seconds * 5 * 4 / 2;
    check(events >= expectMin, "event count " + std::to_string(events)
          + " >= " + std::to_string(expectMin));

    // ── Test 4: double-stop and restart safety ───────────────────────────────
    std::cout << "\n--- Test 4: stop is idempotent, restart works ---\n";
    rec.stopRecording();   // no-op
    check(true, "second stopRecording() is a safe no-op");
    const std::string mkv2 = "/tmp/record_test2.mkv";
    fs::remove(mkv2); fs::remove("/tmp/record_test2.ass");
    check(rec.startRecording(usb->address, rf, mkv2), "restart OK");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    rec.stopRecording();
    check(fs::exists(mkv2) && fs::file_size(mkv2) > 0, "second recording written");

    // ── Test 5: stale telemetry renders as dashes (clock stays live) ─────────
    std::cout << "\n--- Test 5: stale telemetry -> dashes ---\n";
    const std::string mkv3 = "/tmp/record_test3.mkv";
    const std::string ass3 = "/tmp/record_test3.ass";
    fs::remove(mkv3); fs::remove(ass3);
    check(rec.startRecording(usb->address, rf, mkv3), "start (stale case) OK");
    // Every validity flag TRUE, every per-source timestamp OLD.  Ageing only the
    // legacy timestampMs (as this test used to) proves nothing now that the
    // renderer judges each source by its own stamp: with the new flags left at
    // their fail-closed default of false, the overlay dashed for the wrong
    // reason and the test passed even when every per-source timeout was broken.
    for (int t = 0; t < 3 * 5 && rec.isRecording(); ++t) {
        dashcam::record::OverlayData od;
        od.timestampMs         = epochMs();          // legacy field deliberately FRESH
        od.speedKmh            = 42.0f;              // would show if treated as fresh
        od.accelerationMs2     = 3.5f;
        od.headingDeg          = 123.0f;
        od.latitude            = 1.5;
        od.longitude           = 2.5;
        od.speedValid          = true;
        od.accelValid          = true;
        od.positionValid       = true;
        od.headingValid        = true;
        const int64_t old      = epochMs() - 10000;  // 10 s old
        od.speedTimestampMs    = old;
        od.accelTimestampMs    = old;
        od.positionTimestampMs = old;
        od.headingTimestampMs  = old;
        rec.setOverlayData(od);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    rec.stopRecording();

    std::ifstream in3(ass3);
    int dashTL = 0, freshSpd = 0, clockEvents = 0, adasBanner3 = 0;
    int dashHdg = 0, freshHdg = 0, dashPos = 0;
    // Acceleration is checked here too.  It was the ONE of the four domains
    // this test never looked at, so a broken accelTimestampMs would have gone
    // through it untouched — the assertions covered speed, heading and position
    // and simply did not mention the fourth.
    int dashAcc = 0, freshAcc = 0;
    while (std::getline(in3, line)) {
        if (line.find(",TL,,0,0,0,,SPD -- km/h") != std::string::npos) ++dashTL;
        if (line.find(",TL,,0,0,0,,SPD 42")      != std::string::npos) ++freshSpd;
        if (line.find("ACC -- m/s2")             != std::string::npos) ++dashAcc;
        if (line.find("ACC +3.5")                != std::string::npos) ++freshAcc;
        if (line.find(",TR,,0,0,0,,HDG --")      != std::string::npos) ++dashHdg;
        if (line.find(",TR,,0,0,0,,HDG 123")     != std::string::npos) ++freshHdg;
        if (line.find(",BL,,0,0,0,,LAT --")      != std::string::npos) ++dashPos;
        if (line.find(",BR,,0,0,0,,20")          != std::string::npos) ++clockEvents;
        if (line.find(",ADAS,,0,0,0,,")          != std::string::npos) ++adasBanner3;
    }
    check(dashTL > 0,       "per-source stale timestamp renders 'SPD -- km/h'");
    check(freshSpd == 0,    "no fresh speed leaks through on a stale speed stamp");
    check(dashAcc > 0,      "per-source stale timestamp renders 'ACC -- m/s2'");
    check(freshAcc == 0,    "no fresh acceleration leaks through on a stale accel stamp");
    check(dashHdg > 0,      "per-source stale timestamp renders 'HDG --'");
    check(freshHdg == 0,    "no fresh heading leaks through on a stale heading stamp");
    check(dashPos > 0,      "per-source stale timestamp renders 'LAT --'");
    check(clockEvents > 0,  "bottom-right clock still populated while stale");
    check(adasBanner3 == 0, "no ADAS banner emitted when adasValid is false");

    // ── Test 6: sources age INDEPENDENTLY ────────────────────────────────────
    // The defect this guards: a live GNSS fix refreshes speed while a dead ECU
    // leaves acceleration inherited, and a stationary vehicle has a good fix
    // with no trustworthy course.  One shared flag necessarily certifies the
    // stale half, which is fabricated evidence in a recording.
    std::cout << "\n--- Test 6: independent per-source validity ---\n";
    const std::string mkv4 = "/tmp/record_test4.mkv";
    const std::string ass4 = "/tmp/record_test4.ass";
    fs::remove(mkv4); fs::remove(ass4);
    check(rec.startRecording(usb->address, rf, mkv4), "start (mixed case) OK");
    for (int t = 0; t < 3 * 5 && rec.isRecording(); ++t) {
        dashcam::record::OverlayData od;
        const int64_t nowT     = epochMs();
        od.timestampMs         = nowT;
        // Speed live, acceleration dead (GNSS speed with a silent ECU).
        od.speedKmh            = 55.0f;
        od.speedValid          = true;
        od.speedTimestampMs    = nowT;
        od.accelerationMs2     = 9.9f;      // must NOT appear
        od.accelValid          = false;
        // Position live, heading untrustworthy (stationary, or warming up).
        od.latitude            = 10.5;
        od.longitude           = 106.5;
        od.altitudeM           = 12.0;
        od.positionValid       = true;
        od.positionTimestampMs = nowT;
        od.headingDeg          = 90.0f;     // the struct's placeholder; must NOT appear
        od.headingValid        = false;
        rec.setOverlayData(od);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    rec.stopRecording();

    std::ifstream in4(ass4);
    int spdLive = 0, accDash = 0, accLive = 0, hdgDash = 0, hdgLive = 0, posLive = 0;
    while (std::getline(in4, line)) {
        if (line.find(",TL,,0,0,0,,SPD 55") != std::string::npos) ++spdLive;
        if (line.find("ACC -- m/s2")        != std::string::npos) ++accDash;
        if (line.find("ACC +9.9")           != std::string::npos) ++accLive;
        if (line.find(",TR,,0,0,0,,HDG --") != std::string::npos) ++hdgDash;
        if (line.find(",TR,,0,0,0,,HDG 090")!= std::string::npos) ++hdgLive;
        if (line.find(",BL,,0,0,0,,LAT 10.5")!= std::string::npos) ++posLive;
    }
    check(spdLive > 0, "live speed still renders when acceleration is invalid");
    check(accDash > 0, "invalid acceleration renders 'ACC --' beside a live speed");
    check(accLive == 0, "stale acceleration never leaks under a live speed");
    check(posLive > 0, "live position still renders when heading is invalid");
    check(hdgDash > 0, "invalid heading renders 'HDG --' beside a live position");
    check(hdgLive == 0, "placeholder heading never leaks under a live position");

    // ── Test 7: sources AGE independently ────────────────────────────────────
    // Test 6 proves the four validity FLAGS are honoured separately, which is a
    // different property from the four TIMEOUTS being applied separately: every
    // field it marks invalid carries Valid=false, so the timestamps are never
    // what decides.  A renderer that ignored the per-source stamps entirely and
    // keyed staleness off one shared clock would pass Test 6 unchanged.
    //
    // Here every flag is TRUE and only the stamps differ, so the per-domain
    // timeout is the sole thing that can produce a dash.
    std::cout << "\n--- Test 7: independent per-source ageing ---\n";
    const std::string mkv5 = "/tmp/record_test5.mkv";
    const std::string ass5 = "/tmp/record_test5.ass";
    fs::remove(mkv5); fs::remove(ass5);
    check(rec.startRecording(usb->address, rf, mkv5), "start (ageing case) OK");
    for (int t = 0; t < 3 * 5 && rec.isRecording(); ++t) {
        dashcam::record::OverlayData od;
        const int64_t nowT     = epochMs();
        const int64_t oldT     = nowT - 10000;      // 10 s old
        od.timestampMs         = nowT;
        // All four VALID; only the stamps differ.
        od.speedValid = od.accelValid = od.positionValid = od.headingValid = true;
        od.speedKmh            = 61.0f;
        od.speedTimestampMs    = nowT;              // fresh
        od.accelerationMs2     = 7.7f;              // must NOT appear
        od.accelTimestampMs    = oldT;              // aged out
        od.latitude            = 20.25;
        od.longitude           = 100.25;
        od.altitudeM           = 33.0;
        od.positionTimestampMs = nowT;              // fresh
        od.headingDeg          = 210.0f;            // must NOT appear
        od.headingTimestampMs  = oldT;              // aged out
        rec.setOverlayData(od);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    rec.stopRecording();

    std::ifstream in5(ass5);
    int aSpd = 0, aAccDash = 0, aAccLive = 0;
    int aPos = 0, aHdgDash = 0, aHdgLive = 0;
    while (std::getline(in5, line)) {
        if (line.find(",TL,,0,0,0,,SPD 61")  != std::string::npos) ++aSpd;
        if (line.find("ACC -- m/s2")         != std::string::npos) ++aAccDash;
        if (line.find("ACC +7.7")            != std::string::npos) ++aAccLive;
        if (line.find(",BL,,0,0,0,,LAT 20.25")!= std::string::npos) ++aPos;
        if (line.find(",TR,,0,0,0,,HDG --")  != std::string::npos) ++aHdgDash;
        if (line.find(",TR,,0,0,0,,HDG 210") != std::string::npos) ++aHdgLive;
    }
    check(aSpd > 0,      "fresh speed stamp renders while the accel stamp is aged out");
    check(aAccDash > 0,  "aged accel stamp dashes even with accelValid true");
    check(aAccLive == 0, "aged acceleration never leaks beside a fresh speed");
    check(aPos > 0,      "fresh position stamp renders while the heading stamp is aged out");
    check(aHdgDash > 0,  "aged heading stamp dashes even with headingValid true");
    check(aHdgLive == 0, "aged heading never leaks beside a fresh position");

    // ── Test 8: an out-of-range heading is refused ───────────────────────────
    // isfinite() is not sufficient for heading, and this is the case that shows
    // why: 1e30f passes every finiteness check and then divides by 45 into a
    // value no integer type holds, which made std::lround() in the cardinal
    // conversion undefined.  The renderer must dash it rather than print a
    // direction derived from an undefined conversion.
    std::cout << "\n--- Test 8: out-of-range heading -> dash ---\n";
    const std::string mkv6 = "/tmp/record_test6.mkv";
    const std::string ass6 = "/tmp/record_test6.ass";
    fs::remove(mkv6); fs::remove(ass6);
    check(rec.startRecording(usb->address, rf, mkv6), "start (range case) OK");
    for (int t = 0; t < 3 * 5 && rec.isRecording(); ++t) {
        dashcam::record::OverlayData od;
        const int64_t nowT     = epochMs();
        od.timestampMs         = nowT;
        od.speedKmh            = 44.0f;
        od.speedValid          = true;
        od.speedTimestampMs    = nowT;
        // Claimed valid and freshly stamped — only the VALUE is impossible.
        od.headingDeg          = 1e30f;
        od.headingValid        = true;
        od.headingTimestampMs  = nowT;
        rec.setOverlayData(od);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    rec.stopRecording();

    std::ifstream in6(ass6);
    int rSpd = 0, rHdgDash = 0, rHdgAny = 0;
    while (std::getline(in6, line)) {
        if (line.find(",TL,,0,0,0,,SPD 44") != std::string::npos) ++rSpd;
        if (line.find(",TR,,0,0,0,,HDG --") != std::string::npos) ++rHdgDash;
        else if (line.find(",TR,,0,0,0,,HDG") != std::string::npos) ++rHdgAny;
    }
    check(rSpd > 0,      "live speed unaffected by an out-of-range heading");
    check(rHdgDash > 0,  "out-of-range heading renders 'HDG --' despite headingValid");
    check(rHdgAny == 0,  "no numeric heading emitted for an out-of-range value");

    testLiveSegmented(usb->address, rf);

    std::cout << "\n" << (g_fails == 0 ? "RESULT: PASS" : "RESULT: FAIL")
              << " (" << g_fails << " failures)\n";
    return g_fails == 0 ? 0 : 1;
}
