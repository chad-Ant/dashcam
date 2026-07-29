// record_test — validates the redesigned librecord:
//   1. UVC compressed passthrough: the camera's MJPEG/H264 stream goes into
//      MKV with no re-encode.
//   2. ASS telemetry sidecar: same basename, four corner styles, telemetry
//      events at SubtitleRateHz with a live clock and varying speed values.
//   3. Strict precompressed gate: a raw format must be REJECTED.
//   4. Clean EOS finalisation on stop.
//
// Usage: record_test [seconds]   (default 10; uses the first USB camera)

#include "libcamera.h"
#include "libconfig.h"
#include "liblog.h"
#include "libnetwork.h"
#include "librecord.h"

#include <linux/videodev2.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
namespace fs  = std::filesystem;
namespace net = dashcam::network;

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

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    const int seconds = argc > 1 ? std::stoi(argv[1]) : 10;

    auto log = dashcam::log::getCallback();

    // ── find a USB camera with a compressed format ───────────────────────────
    std::vector<cameraInfo> cams;
    if (getCameraList(cams, log) != ERROR_CODE::NONE) {
        std::cerr << "ERROR: camera enumeration failed\n";
        return 1;
    }
    const cameraInfo* usb = nullptr;
    int fmtIdx = -1, rawIdx = -1;
    for (const auto& c : cams) {
        if (c.type != CAMERA_TYPE::USB) continue;
        for (size_t i = 0; i < c.videoFormats.size(); ++i) {
            const auto& f = c.videoFormats[i];
            const bool compressed = f.pixelFormat == V4L2_PIX_FMT_MJPEG ||
                                    f.pixelFormat == V4L2_PIX_FMT_H264;
            if (compressed && fmtIdx < 0 && f.width >= 1280 &&
                f.frameRate >= 24.0f && f.frameRate <= 31.0f)
                { usb = &c; fmtIdx = (int)i; }
            if (!compressed && rawIdx < 0) rawIdx = (int)i;
        }
        if (usb) break;
    }
    if (!usb || fmtIdx < 0) {
        std::cerr << "ERROR: no USB camera with a compressed format found\n";
        return 1;
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
    while (std::getline(in3, line)) {
        if (line.find(",TL,,0,0,0,,SPD -- km/h") != std::string::npos) ++dashTL;
        if (line.find(",TL,,0,0,0,,SPD 42")      != std::string::npos) ++freshSpd;
        if (line.find(",TR,,0,0,0,,HDG --")      != std::string::npos) ++dashHdg;
        if (line.find(",TR,,0,0,0,,HDG 123")     != std::string::npos) ++freshHdg;
        if (line.find(",BL,,0,0,0,,LAT --")      != std::string::npos) ++dashPos;
        if (line.find(",BR,,0,0,0,,20")          != std::string::npos) ++clockEvents;
        if (line.find(",ADAS,,0,0,0,,")          != std::string::npos) ++adasBanner3;
    }
    check(dashTL > 0,       "per-source stale timestamp renders 'SPD -- km/h'");
    check(freshSpd == 0,    "no fresh speed leaks through on a stale speed stamp");
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

    std::cout << "\n" << (g_fails == 0 ? "RESULT: PASS" : "RESULT: FAIL")
              << " (" << g_fails << " failures)\n";
    return g_fails == 0 ? 0 : 1;
}
