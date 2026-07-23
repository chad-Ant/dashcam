// driverstate_test — validates libdriverstate (binary drowsiness ResNet18) three ways:
//
//   driverstate_test <engine> [seconds]              live UVC smoke test (default 20 s)
//   driverstate_test <engine> --video <file> [w h [s]]  file-feed decode validation
//   driverstate_test <engine> --image <file> [s [nofd]]  still image via imagefreeze —
//                                                    face-detect + classify path check;
//                                                    trailing "nofd" disables face
//                                                    detection (classify the full
//                                                    frame — for pre-cropped stills)
//
// Live mode targets the driver-facing USB camera (first USB device with a raw
// YUYV format; lowest native frame rate is preferred since the branch drops to
// 2 fps anyway) and proves the Camera_GST integration end-to-end: branch rate
// cap, centre-crop preprocess, inference thread, clean teardown.
//
// The model's sigmoid POLARITY is unpublished (see libdriverstate.h).  Live
// mode prints P(drowsy) every second — bench-check it once: face the camera
// with eyes open, then keep them closed for a few seconds.  P(drowsy) must
// rise while closed; if it falls instead, set positiveIsDrowsy = false.
// Frames where YuNet finds no face print NO-FACE and the classifier
// stays idle — the operator must actually be in frame for the bench-check.
//
// Video mode feeds a clip through the same detector bin; PASS requires valid
// inference results for most of the run (content is not asserted — clip
// choice is the operator's).
//
// Operator-tunable knobs come from libconfig's DetectionConfig (<Detection>),
// mirroring how the production app consumes them.

#include "libcamera.h"
#include "libcamera_usb.h"
#include "libconfig.h"
#include "libdriverstate.h"

#include <linux/videodev2.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
using namespace dashcam::driver;

static const char* levelStr(FatigueLevel l) {
    switch (l) {
        case FatigueLevel::OK:      return "OK";
        case FatigueLevel::CAUTION: return "CAUTION";
        case FatigueLevel::WARNING: return "WARNING";
        case FatigueLevel::FATIGUE: return "FATIGUE";
    }
    return "?";
}

static void printResult(int t, const DriverStateResult& r) {
    std::cout << "t+" << t << "s  ";
    if (!r.valid) {
        std::cout << "(no result yet)\n";
        return;
    }
    if (!r.faceDetected) {
        std::cout << "NO-FACE (classifier idle)  score="
                  << r.fatigueScore << " (" << levelStr(r.fatigueLevel)
                  << ")\n";
        return;
    }
    // makeDriverConfig leaves positiveIsDrowsy at its library default (false),
    // so drowsyProbability = 1 - sigmoid(logit).  Recover the RAW model output
    // (the value the static-image investigation found stuck at ~1) so a live
    // subject can see whether it actually MOVES between eyes-open / eyes-closed.
    const float rawSigmoid = 1.0f - r.drowsyProbability;
    std::cout << std::fixed << std::setprecision(4)
              << (r.state == DriverState::DROWSY ? "DROWSY " : "NATURAL")
              << "  P(drowsy)=" << r.drowsyProbability
              << "  rawSigmoid=" << rawSigmoid
              << std::setprecision(1)
              << "  score=" << r.fatigueScore
              << " (" << levelStr(r.fatigueLevel) << ")\n"
              << std::defaultfloat;
}

// Fill the library structs from libconfig's <Detection> / <DriverScore>
// sections — the same consumption pattern the production app uses.
static FatigueScoreConfig makeScoreConfig(
        const dashcam::config::DriverScoreConfig& s) {
    FatigueScoreConfig c;
    c.scoreInitial       = (float)s.scoreInitial;
    c.scoreUpper         = (float)s.scoreUpper;
    c.scoreLower         = (float)s.scoreLower;
    c.drowsyChunkSec     = (float)s.drowsyChunkSec;
    c.drowsyChunkPenalty = (float)s.drowsyChunkPenalty;
    c.awakeChunkSec      = (float)s.awakeChunkSec;
    c.awakeChunkReward   = (float)s.awakeChunkReward;
    c.laneDepartThresh   = (float)s.laneDepartThresh;
    c.laneReturnSec      = (float)s.laneReturnSec;
    c.laneDriftPenalty   = (float)s.laneDriftPenalty;
    c.capDecayPerHour    = (float)s.capDecayPerHour;
    c.capDecayFloor      = (float)s.capDecayFloor;
    c.cautionScore       = (float)s.cautionScore;
    c.warningScore       = (float)s.warningScore;
    c.fatigueScore       = (float)s.fatigueScore;
    c.fatigueSustainSec  = (float)s.fatigueSustainSec;
    c.noFaceFreezes      = (bool)s.noFaceFreezes;
    return c;
}

static DriverStateConfig makeDriverConfig(
        const dashcam::config::DetectionConfig& d, const std::string& engine) {
    DriverStateConfig c;
    c.enginePath      = engine.empty() ? (std::string)d.driverEnginePath : engine;
    c.targetHz        = static_cast<uint32_t>((int)d.driverTargetHz);
    c.branchMaxFps    = static_cast<uint32_t>((int)d.driverBranchMaxFps);
    c.drowsyThreshold = (float)d.driverDrowsyThreshold;
    c.faceDetection      = (bool)d.driverFaceDetection;
    c.faceModelPath      = (std::string)d.driverFaceModelPath;
    c.faceScoreThreshold = (float)d.driverFaceScore;
    c.faceDetectScale    = (float)d.driverFaceDetectScale;
    c.score              = makeScoreConfig(dashcam::config::DriverScoreConfig{});
    return c;
}

// ─── video-file mode ─────────────────────────────────────────────────────────

static int runVideoMode(const std::string& engine, const std::string& video,
                        uint32_t w, uint32_t h, int seconds, bool still,
                        bool faceDetect = true) {
    dashcam::config::DetectionConfig det;   // defaults; file feed = system memory
    DriverStateConfig cfg = makeDriverConfig(det, engine);
    cfg.branchMaxFps  = 0;                  // file feeds aren't live-paced
    cfg.faceDetection = cfg.faceDetection && faceDetect;

    DriverStateDetector detector(w, h, cfg);
    GstElement* bin = detector.createBin();
    if (!bin) { std::cerr << "ERROR: createBin failed\n"; return 1; }

    // Stills become a 5 fps live stream via imagefreeze.  Decode in SOFTWARE
    // (jpegdec/pngdec by extension) — decodebin autoplug picks the NVDEC
    // hardware path whose NVMM output imagefreeze cannot consume.  videoscale
    // to 640×480 may distort aspect slightly; the Haar cascade tolerates it
    // and the face crop re-frames the classifier input regardless.
    const bool isPng = video.size() >= 4
        && video.compare(video.size() - 4, 4, ".png") == 0;
    const std::string desc = still
        ? "filesrc location=\"" + video + "\" ! "
          + (isPng ? "pngdec" : "jpegdec") + " ! imagefreeze "
          "! videoconvert ! videoscale "
          "! video/x-raw,format=BGR,width=" + std::to_string(w)
          + ",height=" + std::to_string(h)
          + ",framerate=5/1 ! identity name=feed"
        : "filesrc location=\"" + video + "\" ! decodebin ! videoconvert "
          "! videoscale ! video/x-raw,format=BGR,width=" + std::to_string(w)
          + ",height=" + std::to_string(h) + " ! identity name=feed";
    GError* err = nullptr;
    GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
    if (!pipe || err) {
        std::cerr << "ERROR: pipeline parse: "
                  << (err ? err->message : "?") << "\n";
        if (err) g_error_free(err);
        if (bin) gst_object_unref(bin);
        return 1;
    }

    gst_bin_add(GST_BIN(pipe), bin);
    GstElement* feed = gst_bin_get_by_name(GST_BIN(pipe), "feed");
    if (!feed || !gst_element_link(feed, bin)) {
        std::cerr << "ERROR: could not link feed -> detector bin\n";
        if (feed) gst_object_unref(feed);
        gst_object_unref(pipe);
        return 1;
    }
    gst_object_unref(feed);

    if (gst_element_set_state(pipe, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "ERROR: pipeline refused to play\n";
        gst_object_unref(pipe);
        return 1;
    }
    detector.start();

    GstBus* bus = gst_element_get_bus(pipe);
    int total = 0, withResult = 0;
    bool error = false;

    const int ticks = seconds * 2;                    // poll every 500 ms
    for (int t = 0; t < ticks; ++t) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 500 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (msg) {
            const bool eos = GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS;
            if (!eos) {
                GError* e = nullptr;
                gst_message_parse_error(msg, &e, nullptr);
                std::cerr << "ERROR on bus: " << (e ? e->message : "?") << "\n";
                if (e) g_error_free(e);
                error = true;
            }
            gst_message_unref(msg);
            if (eos) std::cout << "(end of clip)\n";
            break;
        }

        const DriverStateResult r = detector.poll();
        if (t >= 2) {                                 // 1 s pipeline warm-up
            ++total;
            if (r.valid) ++withResult;
        }
        if (t % 2 == 0) printResult(t / 2, r);
    }

    gst_element_set_state(pipe, GST_STATE_NULL);
    detector.stop();
    gst_object_unref(bus);
    gst_object_unref(pipe);

    std::cout << "\npolls=" << total << "  with-result=" << withResult << "\n";
    const bool pass = !error && total > 0
                   && withResult * 2 >= total;        // results in >= 50% of polls
    std::cout << (pass ? "VIDEO MODE: PASS" : "VIDEO MODE: FAIL") << "\n";
    return pass ? 0 : 1;
}

// ─── live-camera mode (driver-facing UVC) ────────────────────────────────────

static int runLiveMode(const std::string& engine, int seconds,
                       bool faceDetect = true, float detectScale = 0.0f) {
    std::vector<cameraInfo> cameras;
    if (getCameraList(cameras) != ERROR_CODE::NONE || cameras.empty()) {
        std::cerr << "ERROR: no cameras found\n";
        return 1;
    }

    // First USB camera with a raw YUYV format; among candidate sizes prefer
    // 640x480 (square-crops to 480² for the 224² model) at the LOWEST native
    // frame rate — the branch drops to 2 fps regardless, so a slow camera
    // mode just saves USB bandwidth and conversion cost.
    const cameraInfo* usb = nullptr;
    int fmtIdx = -1;
    float bestRate = 1e9f;
    for (const auto& c : cameras) {
        if (c.type != CAMERA_TYPE::USB) continue;
        for (size_t i = 0; i < c.videoFormats.size(); ++i) {
            const auto& f = c.videoFormats[i];
            if (f.pixelFormat != V4L2_PIX_FMT_YUYV) continue;
            if (f.width != 640 || f.height != 480) continue;
            if (f.frameRate >= 4.9f && f.frameRate < bestRate) {
                usb = &c; fmtIdx = static_cast<int>(i); bestRate = f.frameRate;
            }
        }
        if (usb) break;
    }
    if (!usb || fmtIdx < 0) {
        std::cerr << "ERROR: no USB camera with YUYV 640x480 found\n";
        return 1;
    }
    const auto& fmt = usb->videoFormats[static_cast<size_t>(fmtIdx)];
    std::cout << "Using camera: " << usb->address << "  YUYV "
              << fmt.width << "x" << fmt.height << "@" << fmt.frameRate
              << " (branch drops to 2 fps)\n";

    Camera_USB cam(*usb);

    dashcam::config::DetectionConfig det;   // <Detection> defaults: 2 Hz / 2 fps
    DriverStateConfig cfg = makeDriverConfig(det, engine);
    // nofd: skip face detection and centre-crop the frame instead, so EVERY
    // frame is classified even with eyes closed / head drooped.  Isolates the
    // model's response to eye state.
    cfg.faceDetection = cfg.faceDetection && faceDetect;
    // Optional YuNet downscale override (0 = use config default 1.0), for
    // comparing detection recall vs CPU at different scales.
    if (detectScale > 0.0f) cfg.faceDetectScale = detectScale;
    std::cout << "face detection: " << (cfg.faceDetection ? "ON" : "OFF")
              << "  YuNet scale: " << cfg.faceDetectScale << "\n";

    DriverStateDetector detector(fmt.width, fmt.height, cfg);
    cam.addBranch("driverstate", detector.createBin(), /*leaky=*/true);

    cam.open();
    cam.setCameraVideoFormat(fmtIdx);
    cam.start();

    cameraStatus st;
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::RUNNING) {
        std::cerr << "ERROR: camera failed to start\n";
        cam.close();
        return 1;
    }
    cam.setCaptureEnabled(false);
    detector.start();

    std::cout << "Running for " << seconds << " seconds...\n"
              << "Polarity bench-check: keep your eyes OPEN, then hold them\n"
              << "CLOSED for ~5 s — P(drowsy) should RISE while closed.\n"
              << "If it falls instead, set DriverStateConfig::positiveIsDrowsy "
              << "= false.\n\n";

    int validPolls = 0;
    for (int t = 1; t <= seconds; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const DriverStateResult r = detector.poll();
        if (r.valid) ++validPolls;
        printResult(t, r);
    }

    cam.stop();       // flushes appsink → inference thread drains
    detector.stop();  // joins inference thread
    cam.close();

    cam.getCameraStatus(st);
    // Allow 3 s of pipeline warm-up before results are expected.
    const bool pass = st.status == CAMERA_STATUS::CLOSED
                   && validPolls >= seconds - 3;
    std::cout << "\nvalid-polls=" << validPolls << "/" << seconds << "\n"
              << "LIVE MODE: " << (pass ? "PASS" : "FAIL")
              << " (pipeline ran, detector polled, teardown "
              << (st.status == CAMERA_STATUS::CLOSED ? "clean" : "dirty")
              << ")\n";
    return pass ? 0 : 1;
}

// ─── fatigue-score selftest (no camera / engine needed) ──────────────────────

// Drives a FatigueScorer through deterministic synthetic timelines at 2 Hz
// and asserts every scoring rule: chunk quantisation, recovery-in-window,
// awake healing, drowsiness-correlated lane drift, no-face freeze, cap decay
// + floor, score floor, the 5-min FATIGUE sustain gate, and reset.
static int runScoreSelftest() {
    using TP = FatigueScorer::TimePoint;
    const TP base = std::chrono::steady_clock::now();
    auto at = [&](double s) {
        return base + std::chrono::milliseconds(static_cast<long long>(s * 1000.0));
    };
    int failures = 0;
    auto check = [&](bool ok, const char* what, float got) {
        std::cout << (ok ? "  PASS  " : "  FAIL  ") << what
                  << "  (got " << got << ")\n";
        if (!ok) ++failures;
    };
    auto eq = [](float a, float b) { return std::fabs(a - b) < 1e-3f; };
    const FatigueScoreConfig cfg;   // library defaults = agreed design values

    { // 1: 25 s continuous drowsiness = 2 completed chunks = -20
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 25.0; t += 0.5) s.update(true, true, true, at(t));
        check(eq(s.score(), 80.0f), "25s drowsy -> 80", s.score());
    }
    { // 2: recovery inside the 10 s window costs nothing
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 8.0;  t += 0.5) s.update(true, true, true,  at(t));
        for (double t = 8.5; t <= 12; t += 0.5) s.update(true, true, false, at(t));
        check(eq(s.score(), 100.0f), "8s drowsy + recovery -> 100 (no deduction)", s.score());
    }
    { // 3: awake healing at +5/10s, capped at the cap
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 25.0; t += 0.5) s.update(true, true, true, at(t));   // 80
        for (double t = 25.5; t <= 70.0; t += 0.5) s.update(true, true, false, at(t)); // +4 chunks
        check(eq(s.score(), 100.0f), "heal 80 -> 100 (capped)", s.score());
    }
    { // 4: drowsiness-correlated drift-and-return = -10, once per episode
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 3.0; t += 0.5) s.update(true, true, true, at(t));
        s.laneOffset(0.9f, true, at(3.0));            // departs (drowsy active)
        s.laneOffset(0.1f, true, at(6.0));            // returns in 3 s -> -10
        s.laneOffset(0.95f, true, at(7.0));           // second drift, same episode
        s.laneOffset(0.0f,  true, at(8.0));           // -> no extra deduction
        for (double t = 3.5; t <= 8.5; t += 0.5) s.update(true, true, true, at(t));
        check(eq(s.score(), 90.0f), "drift+return while drowsy -> -10 once", s.score());
    }
    { // 5: drift while awake never deducts (drowsiness is the trigger)
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 5.0; t += 0.5) s.update(true, true, false, at(t));
        s.laneOffset(0.9f, true, at(2.0));
        s.laneOffset(0.0f, true, at(4.0));
        check(eq(s.score(), 100.0f), "drift while awake -> no deduction", s.score());
    }
    { // 6: return AFTER laneReturnSec = deliberate lane change, no drift penalty
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 2.0; t += 0.5) s.update(true, true, true, at(t));
        s.laneOffset(0.9f, true, at(2.0));            // departs
        for (double t = 2.5; t <= 13.0; t += 0.5) s.update(true, true, true, at(t));
        s.laneOffset(0.0f, true, at(13.0));           // returns after 11 s
        check(eq(s.score(), 90.0f), "late return -> chunk deduction only (90)", s.score());
    }
    { // 7: no-face freezes the score, and ends episodes (partials discarded)
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 25.0; t += 0.5) s.update(true, true, true, at(t));   // 80
        for (double t = 25.5; t <= 55.0; t += 0.5) s.update(true, false, false, at(t));
        check(eq(s.score(), 80.0f), "30s no-face -> frozen at 80", s.score());
        for (double t = 55.5; t <= 66.5; t += 0.5) s.update(true, true, false, at(t));
        check(eq(s.score(), 85.0f), "awake resumes healing -> 85", s.score());
    }
    { // 8: cap decays 10/driving-hour and clamps the score
        FatigueScorer s(cfg, at(0));
        s.update(true, true, false, at(7201.0));      // 2 h in
        check(eq(s.cap(), 80.0f),   "cap after 2 h -> 80", s.cap());
        check(eq(s.score(), 80.0f), "score clamped to cap -> 80", s.score());
    }
    { // 9: cap decay floors at capDecayFloor
        FatigueScorer s(cfg, at(0));
        s.update(true, true, false, at(36000.0));     // 10 h in
        check(eq(s.cap(), 50.0f), "cap after 10 h -> floor 50", s.cap());
    }
    { // 10+11: score floor, FATIGUE sustain gate, and recovery unlatch
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 300.0; t += 0.5) s.update(true, true, true, at(t));
        check(eq(s.score(), -10.0f), "long drowsy -> floored at -10", s.score());
        check(s.level() == FatigueLevel::WARNING ? true
              : s.level() == FatigueLevel::FATIGUE ? false : false,
              "in zone < 5 min -> WARNING (not yet FATIGUE)",
              static_cast<float>(static_cast<int>(s.level())));
        for (double t = 300.5; t <= 460.0; t += 0.5) s.update(true, true, true, at(t));
        check(s.level() == FatigueLevel::FATIGUE,
              "zone sustained > 5 min -> FATIGUE",
              static_cast<float>(static_cast<int>(s.level())));
        for (double t = 460.5; t <= 492.5; t += 0.5) s.update(true, true, false, at(t));
        check(s.level() != FatigueLevel::FATIGUE && s.score() > 0.0f,
              "healing above zone unlatches FATIGUE", s.score());
        // 12: reset restores a fresh session (score, cap AND session clock)
        s.reset(at(500.0));
        check(eq(s.score(), 100.0f) && eq(s.cap(), 100.0f)
                  && s.level() == FatigueLevel::OK,
              "reset -> 100 / cap 100 / OK", s.score());
        s.update(true, true, false, at(500.0 + 3601.0));  // 1 h after reset
        check(eq(s.cap(), 90.0f), "session clock restarted by reset", s.cap());
    }
    { // 13: a lane CROSS whose middle frames lose lane validity still pays —
      // the invalidity holds (not disarms) the drift so the return is confirmed
        FatigueScorer s(cfg, at(0));
        for (double t = 0; t <= 2.0; t += 0.5) s.update(true, true, true, at(t));
        s.laneOffset(0.9f, true,  at(2.0));           // departs (drowsy)
        s.laneOffset(0.0f, false, at(3.0));           // crossing: boundaries lost
        s.laneOffset(0.0f, false, at(4.0));           // still invalid
        s.laneOffset(0.1f, true,  at(5.0));           // back in lane, valid
        for (double t = 2.5; t <= 5.5; t += 0.5) s.update(true, true, true, at(t));
        check(eq(s.score(), 90.0f), "cross with validity gap still deducts -10", s.score());
    }
    { // 14: inconsistent config is CLAMPED, not rejected (never disables us)
        FatigueScoreConfig bad = cfg;
        bad.warningScore  = bad.cautionScore + 1.0f;   // mis-ordered
        bad.drowsyChunkSec = -5.0f;                     // non-positive window
        bool threw = false;
        try {
            FatigueScorer s(bad, at(0));
            // Sanitised: constructs fine (drowsyChunkSec -> 10, thresholds
            // re-ordered) and still scores drowsiness normally: 25 s -> -20.
            for (double t = 0; t <= 25.0; t += 0.5) s.update(true, true, true, at(t));
            check(eq(s.score(), 80.0f),
                  "clamped config still scores drowsiness (-20)", s.score());
        } catch (const std::exception&) { threw = true; }
        check(!threw, "unordered/invalid config -> sanitised, no throw",
              threw ? 0.f : 1.f);
    }

    std::cout << "\nSCORE SELFTEST: " << (failures == 0 ? "PASS" : "FAIL")
              << " (" << failures << " failures)\n";
    return failures == 0 ? 0 : 1;
}

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: driverstate_test <engine.engine> [seconds]\n"
                     "       driverstate_test <engine.engine> --video <file> "
                     "[width height [seconds]]\n"
                     "       driverstate_test --score-selftest\n";
        return 1;
    }
    if (std::strcmp(argv[1], "--score-selftest") == 0)
        return runScoreSelftest();   // pure logic — no GStreamer/camera/engine
    gst_init(&argc, &argv);

    const std::string engine = argv[1];
    if (argc >= 4 && std::strcmp(argv[2], "--video") == 0) {
        const std::string video = argv[3];
        const uint32_t w = argc > 5 ? std::stoul(argv[4]) : 640;
        const uint32_t h = argc > 5 ? std::stoul(argv[5]) : 480;
        const int      s = argc > 6 ? std::stoi(argv[6])  : 30;
        return runVideoMode(engine, video, w, h, s, /*still=*/false);
    }
    if (argc >= 4 && std::strcmp(argv[2], "--image") == 0) {
        const std::string image = argv[3];
        const int  s    = argc > 4 ? std::stoi(argv[4]) : 15;
        const bool nofd = argc > 5 && std::strcmp(argv[5], "nofd") == 0;
        return runVideoMode(engine, image, 640, 480, s, /*still=*/true, !nofd);
    }
    const int  seconds = argc > 2 ? std::stoi(argv[2]) : 20;
    // Trailing args (any order): "nofd" disables face detection; a bare number
    // in (0,1] overrides the YuNet detect scale.  e.g. `<engine> 40 0.5`.
    bool  nofd  = false;
    float scale = 0.0f;
    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "nofd") == 0) { nofd = true; continue; }
        try { scale = std::stof(argv[i]); } catch (...) {}
    }
    return runLiveMode(engine, seconds, !nofd, scale);
}
