// lane_test — validates liblanedetector (UFLD v2 TensorRT):
//
//   lane_test <engine>                          live IMX296 smoke test (10 s)
//   lane_test <engine> --video <file> [w h [s [stride]]]
//                                               real-footage decode validation
//   lane_test --unit [--strict] [--allow-xfail B5,B6] [--list] [filter]
//                                               T0: CPU unit tests (lane_test_unit.cpp)
//   lane_test --gpu --engine E [--tiny T] [--fp16-in E] [--fp16-out E]
//             [--strict] [--allow-xfail ...] [--list] [filter]
//                                               T1/T2: GPU tests (lane_test_gpu.cpp)
//   lane_test --replay --pack DIR --engine E --out DIR
//             [--golden DIR] [--limit N] [--repeat] [--timings]
//                                               deterministic replay (R1)
//
// Live mode targets the IMX296 CSI camera (selected by sysfs sensor name, so
// probe order can't hand us the wrong sensor) and proves the Camera_GST
// integration end-to-end: branch crop to the lower half, 20 fps inlet cap,
// inference thread, clean teardown.  Lane content is not asserted — the bench
// camera faces a room, not a road.
//
// Video mode is the functional check: every (stride'th) frame of a real
// driving clip is cropped to the branch band and run through processFrame()
// — deterministic, unlike the live branch's rate gate — and the decoded
// results must contain lanes for a healthy fraction of the run.
//
// Operator-tunable knobs come from libconfig's DetectionConfig (<Detection>),
// mirroring how the production app consumes them.

#include "libcamera.h"
#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include "liblanedetector.h"
#include "lane_test_common.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
using namespace dashcam::lane;

static const char* directionStr(LaneDirection d) {
    switch (d) {
        case LaneDirection::Straight: return "Straight";
        case LaneDirection::Left:     return "Left";
        case LaneDirection::Right:    return "Right";
        case LaneDirection::UTurn:    return "U-Turn";
    }
    return "Unknown";
}

static void printResult(int t, const LaneResult& r) {
    if (!r.valid) {
        std::cout << "t+" << t << "s  warming up (no valid result)\n";
        return;
    }
    std::cout << "t+" << t << "s  lanes=" << static_cast<int>(r.numLanes);
    if (r.numLanes == 0) {
        std::cout << "  (none detected)";
    } else if (r.currentLaneIndex < 0) {
        std::cout << "  ego=out-of-bounds";
    } else {
        std::cout << "  ego=" << static_cast<int>(r.currentLaneIndex)
                  << "  dir="
                  << directionStr(r.laneAllowedDirections[r.currentLaneIndex]);
        if (r.lateralValid)
            std::cout << "  offset=" << r.lateralOffset;
    }
    std::cout << "\n";
}

// Fill the library struct from libconfig's <Detection> section — the same
// consumption pattern the production app uses.
static LaneDetectorConfig makeLaneConfig(
        const dashcam::config::DetectionConfig& d, const std::string& engine) {
    LaneDetectorConfig c;
    c.enginePath      = engine.empty() ? (std::string)d.laneEnginePath : engine;
    c.targetHz        = static_cast<uint32_t>((int)d.laneTargetHz);
    c.branchMaxFps    = static_cast<uint32_t>((int)d.laneBranchMaxFps);
    c.inputCropTop    = (float)d.laneInputCropTop;
    c.inputCropBottom = (float)d.laneInputCropBottom;
    c.laneReferenceY  = (float)d.laneReferenceY;
    return c;
}

// True if this video node's sensor is the IMX296 (sysfs card name) — same
// probe-order-proof selection dashcam_v0_2 uses.
static bool isImx296(const cameraInfo& info) {
    const std::string::size_type slash = info.address.find_last_of('/');
    const std::string node = (slash == std::string::npos)
                           ? info.address : info.address.substr(slash + 1);
    std::ifstream f("/sys/class/video4linux/" + node + "/name");
    if (!f) return false;
    std::string name((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    return name.find("imx296") != std::string::npos;
}

// ─── video-file mode ─────────────────────────────────────────────────────────
//
// Every decoded frame (or every --stride'th) of the clip, cropped to the
// branch band per geometry() and run through processFrame(): the result set
// depends only on the clip and the engine, not on decode speed (plan bug B16:
// the old pipeline fed a leaky, wall-clock-gated appsink, so a different
// subset of frames was inferred each run).

bool lanetest::runVideoResults(const std::string& engine, const std::string& video,
                               uint32_t w, uint32_t h, int seconds, int stride,
                               std::vector<LaneResult>& out) {
    out.clear();
    dashcam::config::DetectionConfig det;   // <Detection> defaults
    LaneDetectorConfig cfg = makeLaneConfig(det, engine);
    LaneDetector detector(w, h, cfg);
    const LaneGeometry& geo = detector.geometry();

    // decodebin autoplugs nvv4l2decoder, whose NVMM output videoconvert
    // cannot take ("not negotiated").  nvvidconv brings it to system memory
    // and also accepts system memory, should a software decoder be picked.
    const std::string desc =
        "filesrc location=\"" + video + "\" ! decodebin ! nvvidconv "
        "! video/x-raw,format=BGRx ! videoconvert "
        "! videoscale ! video/x-raw,format=BGR,width=" + std::to_string(w)
        + ",height=" + std::to_string(h)
        + " ! appsink name=sink sync=false max-buffers=4 drop=false emit-signals=false";
    GError* err = nullptr;
    GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
    if (!pipe || err) {
        std::cerr << "ERROR: pipeline parse: " << (err ? err->message : "?") << "\n";
        if (err) g_error_free(err);
        if (pipe) gst_object_unref(pipe);
        return false;
    }
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipe), "sink");
    if (gst_element_set_state(pipe, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "ERROR: pipeline refused to play\n";
        gst_object_unref(sink);
        gst_object_unref(pipe);
        return false;
    }

    const size_t rowBytes = static_cast<size_t>(geo.branchW) * 3;
    std::vector<uint8_t> band(rowBytes * geo.branchH);
    const GstClockTime limit = seconds > 0 ? static_cast<GstClockTime>(seconds) * GST_SECOND
                                           : GST_CLOCK_TIME_NONE;
    bool ok = true;
    for (long n = 0;; ++n) {
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) break;                                  // EOS (or error: checked below)
        GstBuffer* buf = gst_sample_get_buffer(sample);
        const GstClockTime pts = buf ? GST_BUFFER_PTS(buf) : GST_CLOCK_TIME_NONE;
        if (GST_CLOCK_TIME_IS_VALID(limit) && GST_CLOCK_TIME_IS_VALID(pts) && pts >= limit) {
            gst_sample_unref(sample);
            break;
        }
        GstVideoInfo info;
        GstVideoFrame frame;
        if (n % std::max(1, stride) == 0 && buf
            && gst_video_info_from_caps(&info, gst_sample_get_caps(sample))
            && gst_video_frame_map(&frame, &info, buf, GST_MAP_READ)) {
            // Rows [cropTopPx, cropTopPx + branchH), honouring the stride.
            const auto* src = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
            const int srcStride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
            for (uint32_t y = 0; y < geo.branchH; ++y)
                std::memcpy(&band[y * rowBytes],
                            src + static_cast<size_t>(geo.cropTopPx + y) * srcStride, rowBytes);
            gst_video_frame_unmap(&frame);
            LaneResult r;
            detector.processFrame(band.data(), band.size(),
                                  GST_CLOCK_TIME_IS_VALID(pts) ? static_cast<int64_t>(pts) : -1, r);
            out.push_back(r);
        }
        gst_sample_unref(sample);
    }

    GstBus* bus = gst_element_get_bus(pipe);
    if (GstMessage* msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR)) {
        GError* e = nullptr;
        gst_message_parse_error(msg, &e, nullptr);
        std::cerr << "ERROR on bus: " << (e ? e->message : "?") << "\n";
        if (e) g_error_free(e);
        gst_message_unref(msg);
        ok = false;
    }
    gst_object_unref(bus);
    gst_element_set_state(pipe, GST_STATE_NULL);
    gst_object_unref(sink);
    gst_object_unref(pipe);
    return ok;
}

static int runVideoMode(const std::string& engine, const std::string& video,
                        uint32_t w, uint32_t h, int seconds, int stride) {
    std::vector<LaneResult> results;
    const bool ok = lanetest::runVideoResults(engine, video, w, h, seconds, stride, results);
    long total = 0, withLanes = 0, withEgo = 0;
    for (size_t i = 0; i < results.size(); ++i) {
        const LaneResult& r = results[i];
        if (!r.valid) continue;
        ++total;
        withLanes += r.numLanes >= 1;
        withEgo   += r.currentLaneIndex >= 0;
        if (i % 20 == 0) printResult(static_cast<int>(i), r);
    }
    std::cout << "\nframes=" << results.size() << "  valid=" << total << "  with-lanes=" << withLanes
              << "  with-ego-lane=" << withEgo << "\n";
    const bool pass = ok && total > 0
                   && withLanes * 2 >= total          // lanes in >= 50% of frames
                   && withEgo   * 3 >= total;         // ego lane in >= 33%
    std::cout << (pass ? "VIDEO MODE: PASS" : "VIDEO MODE: FAIL") << "\n";
    return pass ? 0 : 1;
}

// ─── live-camera mode (IMX296 on CSI) ────────────────────────────────────────

static int runLiveMode(const std::string& engine) {
    std::vector<cameraInfo> cameras;
    if (getCameraList(cameras) != ERROR_CODE::NONE || cameras.empty()) {
        std::cerr << "ERROR: no cameras found\n";
        return 1;
    }

    const cameraInfo* info = nullptr;
    for (const auto& c : cameras)
        if (c.type == CAMERA_TYPE::CSI && isImx296(c)) { info = &c; break; }
    if (!info || info->videoFormats.empty()) {
        std::cerr << "ERROR: no IMX296 CSI camera found\n";
        return 1;
    }

    std::cout << "Using camera: " << info->address
              << "  (IMX296, Argus sensor-id " << info->deviceId << ")\n";

    Camera_CSI cam(*info);

    dashcam::config::DetectionConfig det;   // <Detection> defaults: 20 Hz,
    LaneDetectorConfig cfg = makeLaneConfig(det, engine);   // crop 0.5, 20 fps
    cfg.sourceIsNVMM = true;

    const uint32_t srcW = info->videoFormats[0].width;
    const uint32_t srcH = info->videoFormats[0].height;

    LaneDetector detector(srcW, srcH, cfg);
    cam.addBranch("lanes", detector.createBin(), /*leaky=*/true);

    cam.open();
    cam.setCameraVideoFormat(0);
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

    std::cout << "Running for 10 seconds...\n\n";
    int validPolls = 0;
    std::vector<lanetest::LivePoll> polls;
    for (int t = 1; t <= 10; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const LaneResult result = detector.poll();
        if (result.valid) ++validPolls;
        polls.push_back({ result.valid, result.sequence, detector.hasFreshResult(500) });
        printResult(t, result);
    }

    cam.stop();       // flushes appsink → inference thread drains
    detector.stop();  // joins inference thread
    cam.close();

    cam.getCameraStatus(st);
    const bool pass = lanetest::liveSmokeVerdict(polls, st.status == CAMERA_STATUS::CLOSED,
                                                 detector.processedFrameCount());
    std::cout << "\nvalid-polls=" << validPolls << "/10"
              << " processed=" << detector.processedFrameCount() << "\n";
    // Instrumentation readout (report only): rate gate and frame timing.
    const LaneStats ls = detector.stats();
    const LaneTiming t = detector.poll().timing;
    std::cout << "stats: pulled=" << ls.pulled << " gated=" << ls.gated
              << " processed=" << ls.processed << " errors(map/size/h2d/pre/trt/d2h/sync)="
              << ls.mapFail << "/" << ls.sizeMismatch << "/" << ls.h2dFail << "/"
              << ls.preFail << "/" << ls.trtFail << "/" << ls.d2hFail << "/" << ls.syncFail << "\n";
    std::cout << "last frame: capture->pull "
              << (t.captureMonoNs >= 0 ? std::to_string((t.pullMonoNs - t.captureMonoNs) / 1000000.0) + " ms"
                                       : std::string("unknown (pipeline clock not monotonic)"))
              << ", pull->done " << (t.doneMonoNs - t.pullMonoNs) / 1000000.0 << " ms\n";
    std::cout << "\nLIVE MODE: " << (pass ? "PASS" : "FAIL")
              << " (live inference verified, teardown "
              << (pass ? "clean" : "dirty") << ")\n";
    return pass ? 0 : 1;
}


// A clean teardown alone is not evidence that Argus delivered a frame.
// Allow three seconds of warm-up, then require live inference results — and
// that they ARE live: valid polls alone pass a detector whose results froze
// after the first frame (plan bug B17).  So the sequence must advance between
// >= 90% of consecutive valid polls, and >= 90% of them must be fresh
// (hasFreshResult(500)).
bool lanetest::liveSmokeVerdict(const std::vector<LivePoll>& polls, bool cameraClosed,
                                uint64_t processed) {
    int valid = 0, fresh = 0, transitions = 0, advanced = 0;
    uint64_t prev = 0;
    for (const auto& p : polls) {
        if (!p.valid) continue;
        if (valid > 0) {
            ++transitions;
            advanced += p.sequence > prev;
        }
        prev = p.sequence;
        ++valid;
        fresh += p.fresh;
    }
    return cameraClosed && processed > 0 && valid >= 7
        && transitions > 0 && advanced * 10 >= transitions * 9
        && fresh * 10 >= valid * 9;
}

// ─── replay mode (R1 golden) ─────────────────────────────────────────────────
//
// A pack is a directory of full-size source frames (PNG/JPEG, BGR as decoded
// for the camera — make_pack.py) plus pack.csv:
//
//   # camera=csi            leading "# key=value" lines: src_w, src_h,
//   # crop_top=0.5          crop_top, crop_bottom, ref_y (optional; the
//   file,src_file,src_pts_ns,condition      <Detection> defaults otherwise)
//   00000.png,p00_00010.mkv,123456,night
//
// Each frame is cropped to the branch band exactly as the GStreamer branch
// would (rows [cropTopPx, cropTopPx + branchH)) and run through
// processFrame().  OUT/frames.csv gets one row per frame with the decoded
// result and FNV-1a hashes of the input and output tensors, so --golden can
// demand bit-identical tensors as well as identical decoded fields.

namespace {

struct Pack {
    std::map<std::string, std::string> meta;
    std::vector<std::map<std::string, std::string>> frames;
};

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) out.push_back(cell);
    if (!line.empty() && line.back() == ',') out.emplace_back();
    return out;
}

// Header row + rows → maps; "# key=value" lines → meta.  False if unreadable.
bool readCsv(const std::string& path, Pack& pack) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    std::vector<std::string> header;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (line[0] == '#') {
            const auto eq = line.find('=');
            if (eq == std::string::npos) continue;
            std::string key = line.substr(1, eq - 1);
            key.erase(0, key.find_first_not_of(' '));
            pack.meta[key] = line.substr(eq + 1);
            continue;
        }
        const auto cells = splitCsv(line);
        if (header.empty()) { header = cells; continue; }
        std::map<std::string, std::string> row;
        for (size_t i = 0; i < header.size(); ++i)
            row[header[i]] = i < cells.size() ? cells[i] : std::string();
        pack.frames.push_back(std::move(row));
    }
    return !header.empty();
}

uint64_t fnv1a(const std::vector<float>& v, uint64_t h = 1469598103934665603ull) {
    const auto* p = reinterpret_cast<const unsigned char*>(v.data());
    for (size_t i = 0; i < v.size() * sizeof(float); ++i) {
        h ^= p[i];
        h *= 1099511628211ull;
    }
    return h;
}

std::string fmtFloat(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
    return buf;
}

std::string hex64(uint64_t v) {
    char buf[20];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
    return buf;
}

using lanetest::insideRepo;

// Columns R1 compares, when present in both runs (a golden recorded before a
// column existed simply doesn't constrain it).  Timing columns never compare.
const std::vector<std::string>& goldenCols() {
    static const std::vector<std::string> cols = [] {
        std::vector<std::string> c = { "valid", "num_lanes", "ego", "lat_off",
                                       "lat_valid", "in_hash", "out_hash", "nonfinite" };
        for (int k = 0; k < 4; ++k)
            for (const char* f : { "det", "head", "npts", "slope", "icpt", "xref",
                                   "ymin", "ymax", "rms" })
                c.push_back("s" + std::to_string(k) + "_" + f);
        return c;
    }();
    return cols;
}

double median(std::vector<double> v) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// The golden is read BEFORE the run writes anything (runReplayMode), so a
// golden that is also --out can't be compared with itself.  With --limit N
// only the golden's first N rows (idx < N) are required.
int compareGolden(const Pack& g, const std::string& outCsv, long limit) {
    Pack o;
    readCsv(outCsv, o);
    std::map<std::string, const std::map<std::string, std::string>*> byFile;
    for (const auto& r : g.frames) byFile[r.at("file")] = &r;

    int compared = 0, mismatched = 0, missing = 0;
    for (const auto& r : o.frames) {
        const auto it = byFile.find(r.at("file"));
        if (it == byFile.end()) { ++missing; continue; }
        ++compared;
        std::string diffs;
        for (const std::string& col : goldenCols()) {
            const auto a = it->second->find(col);
            const auto b = r.find(col);
            if (a == it->second->end() || b == r.end()) continue;
            if (a->second != b->second)
                diffs += std::string(" ") + col + " " + a->second + "->" + b->second;
        }
        if (!diffs.empty()) {
            if (++mismatched <= 10)
                std::cout << "  DIFF " << r.at("file") << ":" << diffs << "\n";
        }
    }
    size_t required = 0;
    for (const auto& r : g.frames)
        if (limit <= 0 || std::stol(r.at("idx")) < limit) ++required;
    const bool pass = compared > 0 && mismatched == 0 && missing == 0
                   && static_cast<size_t>(compared) == required;
    std::cout << "R1 golden: compared=" << compared << "/" << required
              << (limit > 0 ? " (limited)" : "")
              << " mismatched=" << mismatched << " not-in-golden=" << missing << "\n";
    return pass ? 0 : 1;
}

} // namespace

static int runReplayMode(const std::string& packDir, const std::string& engine,
                         const std::string& outDir, const std::string& goldenDir,
                         long limit, bool repeat, bool timings) {
    namespace fs = std::filesystem;
    Pack pack;
    if (!readCsv(packDir + "/pack.csv", pack)) {
        std::cerr << "ERROR: cannot read " << packDir << "/pack.csv\n";
        return 1;
    }
    if (insideRepo(outDir)) {
        std::cerr << "ERROR: --out " << outDir << " is inside the repo; refusing\n";
        return 1;
    }
    Pack golden;
    if (!goldenDir.empty()) {
        std::error_code ec;
        if (fs::weakly_canonical(fs::absolute(goldenDir), ec)
                == fs::weakly_canonical(fs::absolute(outDir), ec)) {
            std::cerr << "ERROR: --golden and --out are the same directory\n";
            return 1;
        }
        if (!readCsv(goldenDir + "/frames.csv", golden)) {
            std::cerr << "ERROR: cannot read golden " << goldenDir << "/frames.csv\n";
            return 1;
        }
    }
    fs::create_directories(outDir);

    auto metaF = [&](const char* k, float def) {
        const auto it = pack.meta.find(k);
        return it == pack.meta.end() ? def : std::stof(it->second);
    };
    dashcam::config::DetectionConfig det;
    LaneDetectorConfig cfg = makeLaneConfig(det, engine);
    cfg.inputCropTop    = metaF("crop_top", cfg.inputCropTop);
    cfg.inputCropBottom = metaF("crop_bottom", cfg.inputCropBottom);
    cfg.laneReferenceY  = metaF("ref_y", cfg.laneReferenceY);
    cfg.collectTimings  = timings;
    const uint32_t srcW = static_cast<uint32_t>(metaF("src_w", 0));
    const uint32_t srcH = static_cast<uint32_t>(metaF("src_h", 0));
    if (srcW == 0 || srcH == 0) {
        std::cerr << "ERROR: pack.csv needs # src_w= and # src_h=\n";
        return 1;
    }

    LaneDetector detector(srcW, srcH, cfg);
    const LaneGeometry& geo = detector.geometry();
    {
        std::ofstream gf(outDir + "/geometry.txt");
        gf << "src " << geo.srcW << "x" << geo.srcH
           << "\ncrop_top_px " << geo.cropTopPx << "\ncrop_bottom_px " << geo.cropBottomPx
           << "\nbranch " << geo.branchW << "x" << geo.branchH
           << "\nkept_frac " << fmtFloat(geo.keptFrac) << "\neff_ratio " << fmtFloat(geo.effRatio)
           << "\nresized_h " << geo.resizedH << "\ncrop_top_rows " << geo.cropTopRows
           << "\npre_y_scale " << fmtFloat(geo.preYScale) << "\npre_y_off " << fmtFloat(geo.preYOff)
           << "\nimplied_h " << geo.impliedH << "\nimplied_top_src " << geo.impliedTopSrc
           << "\nref_row_src " << geo.refRowSrc
           << "\nanchor_rows_src " << geo.anchorRowSrcFirst << " " << geo.anchorRowSrcLast
           << "\nengine " << engine << "\n";
    }
    std::cout << "replay: " << pack.frames.size() << " frames, src " << srcW << "x" << srcH
              << " -> band rows [" << geo.cropTopPx << ", " << geo.cropTopPx + geo.branchH
              << "), ref row " << geo.refRowSrc << "\n";

    std::ofstream csv(outDir + "/frames.csv");
    csv << "idx,file,ok,valid,seq,num_lanes,ego,lat_off,lat_valid,in_hash,out_hash,nonfinite";
    for (int k = 0; k < 4; ++k)
        for (const char* f : { "det", "head", "npts", "slope", "icpt", "xref", "ymin", "ymax", "rms" })
            csv << ",s" << k << "_" << f;
    csv << ",h2d_us,pre_us,trt_us,d2h_us,dec_us,e2e_us\n";
    std::vector<double> tH2d, tPre, tTrt, tD2h, tDec, tE2e;

    long n = 0, ok = 0, withLanes = 0, nondet = 0, unreadable = 0;
    LaneDebugTensors dbg, dbg2;
    for (const auto& fr : pack.frames) {
        if (limit > 0 && n >= limit) break;
        const std::string file = fr.at("file");
        const cv::Mat img = cv::imread(packDir + "/" + file, cv::IMREAD_COLOR);
        if (img.empty() || img.cols != static_cast<int>(srcW)
                        || img.rows != static_cast<int>(srcH)) {
            std::cerr << "ERROR: " << file << " unreadable or not "
                      << srcW << "x" << srcH << "\n";
            ++unreadable;
            ++n;
            continue;
        }
        const cv::Mat band = img.rowRange(static_cast<int>(geo.cropTopPx),
                                          static_cast<int>(geo.cropTopPx + geo.branchH)).clone();
        const int64_t pts = fr.count("src_pts_ns") && !fr.at("src_pts_ns").empty()
                          ? std::stoll(fr.at("src_pts_ns")) : -1;

        LaneResult r;
        const bool good = detector.processFrame(band.data, band.total() * band.elemSize(),
                                                pts, r, &dbg);
        uint64_t hIn = fnv1a(dbg.input), hOut = 1469598103934665603ull;
        for (const auto& o : dbg.outputs) hOut = fnv1a(o, hOut);
        if (repeat) {
            LaneResult r2;
            detector.processFrame(band.data, band.total() * band.elemSize(), pts, r2, &dbg2);
            uint64_t h2 = 1469598103934665603ull;
            for (const auto& o : dbg2.outputs) h2 = fnv1a(o, h2);
            if (fnv1a(dbg2.input) != hIn || h2 != hOut
                || fmtFloat(r2.lateralOffset) != fmtFloat(r.lateralOffset)
                || r2.numLanes != r.numLanes || r2.currentLaneIndex != r.currentLaneIndex)
                ++nondet;
        }
        ok += good;
        withLanes += r.numLanes >= 1;
        csv << n << "," << file << "," << good << "," << r.valid << "," << r.sequence << ","
            << static_cast<int>(r.numLanes) << "," << static_cast<int>(r.currentLaneIndex) << ","
            << (r.lateralValid ? fmtFloat(r.lateralOffset) : std::string()) << ","
            << r.lateralValid << "," << hex64(hIn) << "," << hex64(hOut) << ","
            << r.nonFinite;
        for (const auto& sl : r.slots)
            csv << "," << sl.detected << "," << static_cast<int>(sl.head) << "," << sl.nPts
                << "," << fmtFloat(sl.slope) << "," << fmtFloat(sl.intercept)
                << "," << fmtFloat(sl.xRef) << "," << fmtFloat(sl.yMin)
                << "," << fmtFloat(sl.yMax) << "," << fmtFloat(sl.rmsPx);
        const auto& t = r.timing;
        const double e2e = t.doneMonoNs >= 0 && t.pullMonoNs >= 0
                         ? (t.doneMonoNs - t.pullMonoNs) / 1000.0 : -1;
        csv << "," << fmtFloat(t.h2dUs) << "," << fmtFloat(t.preUs) << "," << fmtFloat(t.trtUs)
            << "," << fmtFloat(t.d2hUs) << "," << fmtFloat(t.decUs) << "," << e2e << "\n";
        if (good && timings) {
            tH2d.push_back(t.h2dUs); tPre.push_back(t.preUs); tTrt.push_back(t.trtUs);
            tD2h.push_back(t.d2hUs); tDec.push_back(t.decUs); tE2e.push_back(e2e);
        }
        ++n;
    }
    csv.close();

    std::cout << "frames=" << n << " ok=" << ok << " with-lanes=" << withLanes
              << " unreadable=" << unreadable;
    if (repeat) std::cout << " nondeterministic=" << nondet;
    std::cout << "\n";
    if (timings)
        std::cout << "stage medians (us): h2d " << median(tH2d) << "  pre " << median(tPre)
                  << "  trt " << median(tTrt) << "  d2h " << median(tD2h)
                  << "  dec " << median(tDec) << "  e2e(incl. debug copies) " << median(tE2e) << "\n";
    const LaneStats st = detector.stats();
    std::cout << "stats: processed=" << st.processed << " nonfinite=" << st.nonFinite
              << " h2d/pre/trt/d2h/sync fail=" << st.h2dFail << "/" << st.preFail << "/"
              << st.trtFail << "/" << st.d2hFail << "/" << st.syncFail << "\n";

    int rc = (ok == n - unreadable && unreadable == 0 && nondet == 0) ? 0 : 1;
    if (!goldenDir.empty()) {
        if (compareGolden(golden, outDir + "/frames.csv", limit) != 0) rc = 1;
    }
    std::cout << "RESULT: " << (rc == 0 ? "PASS" : "FAIL") << "\n";
    return rc;
}

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: lane_test <engine.engine>\n"
                     "       lane_test <engine.engine> --video <file> "
                     "[width height [seconds [stride]]]\n"
                     "       lane_test --unit [--strict] [--allow-xfail B5,..] [--list] [filter]\n"
                     "       lane_test --gpu --engine E [--tiny T] [--fp16-in E] [--fp16-out E] "
                     "[--strict] [--allow-xfail ..] [--list] [filter]\n"
                     "       lane_test --replay --pack DIR --engine E --out DIR "
                     "[--golden DIR] [--limit N] [--repeat] [--timings]\n";
        return 1;
    }
    gst_init(&argc, &argv);

    if (std::strcmp(argv[1], "--child-b19") == 0 && argc >= 3)   // L12's child (internal)
        return lanetest::childB19(argv[2]);

    if (std::strcmp(argv[1], "--unit") == 0 || std::strcmp(argv[1], "--gpu") == 0) {
        const bool gpu = std::strcmp(argv[1], "--gpu") == 0;
        lanetest::TestEnv env;
        std::string filter;
        bool strict = false, list = false;
        std::set<std::string> allow;
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(1); }
                return argv[++i];
            };
            if      (a == "--strict")      strict = true;
            else if (a == "--list")        list = true;
            else if (a == "--engine")      env.engine = next();
            else if (a == "--tiny")        env.tinyEngine = next();
            else if (a == "--fp16-in")     env.fp16InEngine = next();
            else if (a == "--fp16-out")    env.fp16OutEngine = next();
            else if (a == "--allow-xfail") {
                std::stringstream ss(next());
                std::string id;
                while (std::getline(ss, id, ',')) if (!id.empty()) allow.insert(id);
            }
            else if (a.rfind("--", 0) == 0) { std::cerr << "unknown option " << a << "\n"; return 1; }
            else filter = a;
        }
        if (gpu && env.engine.empty() && !list) {
            std::cerr << "--gpu needs --engine\n";
            return 1;
        }
        return lanetest::runTests(gpu ? "gpu" : "unit",
                                  gpu ? lanetest::gpuTests() : lanetest::unitTests(),
                                  env, filter, strict, allow, list);
    }

    if (std::strcmp(argv[1], "--replay") == 0) {
        std::string pack, engine, out, golden;
        long limit = 0;
        bool repeat = false, timings = false;
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(1); }
                return argv[++i];
            };
            if      (a == "--pack")   pack   = next();
            else if (a == "--engine") engine = next();
            else if (a == "--out")    out    = next();
            else if (a == "--golden") golden = next();
            else if (a == "--limit")  limit  = std::stol(next());
            else if (a == "--repeat") repeat = true;
            else if (a == "--timings") timings = true;
            else { std::cerr << "unknown option " << a << "\n"; return 1; }
        }
        if (pack.empty() || engine.empty() || out.empty()) {
            std::cerr << "--replay needs --pack, --engine and --out\n";
            return 1;
        }
        return runReplayMode(pack, engine, out, golden, limit, repeat, timings);
    }

    const std::string engine = argv[1];
    if (argc >= 4 && std::strcmp(argv[2], "--video") == 0) {
        const std::string video = argv[3];
        const uint32_t w = argc > 5 ? std::stoul(argv[4]) : 1280;
        const uint32_t h = argc > 5 ? std::stoul(argv[5]) : 720;
        const int      s = argc > 6 ? std::stoi(argv[6])  : 60;
        const int stride = argc > 7 ? std::stoi(argv[7])  : 1;
        return runVideoMode(engine, video, w, h, s, stride);
    }
    return runLiveMode(engine);
}
