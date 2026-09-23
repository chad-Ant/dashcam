// record_test — validates librecord:
//   Part A (no hardware, always runs; synthetic videotestsrc sources):
//     v0.3 single-file pipeline strings + ASS lines unchanged (golden), segment
//     naming, segmented MJPEG/H.264 (gapless, t=0 starts, per-segment sidecars
//     covering each segment's start), EOS before the first frame (no abort),
//     stall watchdog, mid-stream source error still finalises, loop-overwrite
//     retention safety, luma tap for software auto-exposure.
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

#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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
};

static GstPadProbeReturn countProbe(GstPad*, GstPadProbeInfo* info, gpointer user) {
    auto* mi = static_cast<MkvInfo*>(user);
    GstBuffer* b = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!b) return GST_PAD_PROBE_OK;
    ++mi->frames;
    if (GST_BUFFER_PTS_IS_VALID(b)) {
        const int64_t pts = static_cast<int64_t>(GST_BUFFER_PTS(b));
        if (mi->firstPts < 0 || pts < mi->firstPts) mi->firstPts = pts;
        if (pts > mi->lastPts) mi->lastPts = pts;
    }
    return GST_PAD_PROBE_OK;
}

static MkvInfo probeMkv(const std::string& path) {
    MkvInfo mi;
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
        allAss  = allAss && ai.exists && ai.styles == 5;
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
    check(allAss, "every segment has its own .ass (5 styles)");
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
    check(styles == 5, "five styles (TL/TR/BL/BR + ADAS banner)");
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
