// lane_drive — the in-car / bench harness of the lane test suite (plan P5a +
// P5b, ~/drive_logs/lane_test_suite_plan_20261025.md sections 3.3, 3.4, 10, 12).
//
//   lane_drive --cam csi|usb --out RUN_DIR [options]       one camera per process
//   lane_drive --mount-replay --out DIR --files A.mkv,B.mkv [...]   mount monitor over footage (B9)
//
// Runs liblanedetector live on one camera and logs everything a drive needs
// to be analysed afterwards, under RUN_DIR/<cam>/:
//   frames.csv       every result: timestamps, stage timings, lanes, per-slot geometry, band luma,
//                    slot order, dump id, live CAN (lagged 2-3 s; convenience only)
//   counters.csv     1 Hz: source fps, lane rate, gate/drops, errors, clip ring / clips / dumps,
//                    process RSS/threads/fds
//   camera_ctrl.csv  1 Hz: exposure / gain / focus (UGREEN; CSI: unavailable, see P0)
//   image_stats.csv  --stats-hz (2): band / sky / full luma, noise, motion
//   mount.csv        1 Hz: edge-based view shift vs the parked reference (plan V2)
//   clock.csv        1 Hz: CLOCK_MONOTONIC / REALTIME / BOOTTIME triples (CAN join, steps)
//   events.csv       lifecycle, faults, triggers, mount, degrade
//   review/*.mkv     review footage, 60 s segments (CSI JPEG --review-fps 5, UGREEN MJPEG 10);
//                    review/<cam>_p<part>.ass: 5 Hz lane banner on the MKV timeline
//   clips/*.mkv      full-rate clips kept from the in-RAM clip ring around events (clips_index.csv)
//   dumps/           output tensors (.bin) + input band (.jpg) + the camera frame with the same
//                    PTS (.jpg), periodic (--tensor-hz) and on events (dumps_index.csv)
//   status.json      1 Hz, for lane_supervisor.py;  run.json  options, engine, geometry;
//   summary.json     totals at the end
//
// CSI: the production Camera_CSI (native 1456x1088@60, capture branch off), and
// three branches: lanes; encode (30 fps nvjpegenc → review at --review-fps +
// clip ring); stats (GRAY8 364x272 at --stats-hz via VIC).  UGREEN: a
// pipeline of its own — MJPEG passthrough for review and for the clip ring,
// decoded for lanes by jpegdec (--usb-decode sw, the default) or nvv4l2decoder
// (hw), a --stats-hz tap that also feeds v0.4's frame-rate-priority exposure
// loop; focus held at --focus (690) — both handed back at exit.  UGREEN
// buffers are restamped on arrival and decimated by a probe (no videorate on
// the lane and review paths; see restampProbe / Decimator).
// KNOWN BROKEN (2026-10-25): --usb-decode hw delivers 13.4 Hz for 20 with a
// latency that grows ~1 s per 3 s, also with enable-max-performance /
// disable-dpb; sw measured 20.0 Hz, p50 43 ms.
//
// Clips: triggers (stall / camera error / latency spike / error counter /
// mount shift = fault; RUN_DIR/triggers.csv lines from can_live.py and
// `lane_run.sh mark`; flicker bursts and a random 10 s every 10 min) open a
// window [t − 10 s, t + 10 s] that the ring's frames fill; a token bucket
// (--clip-budget-gb-h) keeps faults longest when the budget binds.
// Degrade ladder (RUN_DIR/ctl/degrade, written by lane_supervisor.py):
// 1 clips off, 2 dumps off, 3 CSI review off, 4 UGREEN lanes off, 5 UGREEN
// review off (CSVs only).  RUN_DIR/ctl/<cam>_lane_hz lowers the UGREEN lane rate.
//
// The first --lanes-off-s seconds run with the lane branch closed (baseline
// for the resource deltas; --lanes-schedule off:S,on:S,... instead for B0).
// A camera error or a 10 s stall restarts the camera in-process (UGREEN 5
// tries, CSI 2; 2/4/8/16/30 s backoff), then cam_lost: finalise and exit 3,
// so lane_supervisor.py restarts the container (--instance K).  SIGINT /
// SIGTERM: finalise (EOS → MKVs closed, clips and dumps written, CSVs flushed,
// controls handed back) and exit 0.  A finalising watchdog writes summary.json
// with stop_hang and exits 4 if that takes over 30 s.  RUN_DIR must be outside
// the repo (public).

#include "libcamera.h"
#include "libcamera_csi.h"
#include "libcamera_exposure.h"
#include "libcamera_focus.h"
#include "lane_drive_aux.h"
#include "lane_eval_common.h"
#include "lane_test_common.h"
#include "liblanedetector.h"
#include "liblog.h"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <linux/videodev2.h>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace dashcam::camera;
using namespace dashcam::lane;
using namespace lanedrive;
using laneeval::CsvWriter;
using laneeval::fmtF;
using laneeval::monoNs;
using laneeval::realNs;
using laneeval::bootNs;

namespace {

std::atomic<int> g_signal{0};
void onSignal(int sig) { g_signal = sig; }

constexpr int64_t kSec = 1'000'000'000LL;

// ─── options ─────────────────────────────────────────────────────────────────

struct Options {
    std::string cam;                         // csi | usb
    std::string out;                         // RUN_DIR
    std::string engine = "models/culane_res18_fp16.engine";
    float cropTop = -1, cropBottom = -1;     // per-camera defaults below
    float refY = 0.90f;
    uint32_t hz = 20, branchFps = 20;
    std::string rateGate = "pts";            // the suite measures an honest 20 Hz (plan 5.3)
    int reviewFps = -1;                      // csi 5, usb 10
    int jpegQ = 75;
    std::string usbDev = "/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0";
    std::string usbDecode = "sw";            // sw | hw (hw: see the file comment)
    std::string usbExposure = "framerate";   // framerate | camera
    int targetLuma = 110;
    int focus = 690;                         // -1 = leave autofocus alone
    int usbW = 1920, usbH = 1080, usbFps = 30;
    double minutes = 0;                      // 0 = until SIGINT/SIGTERM
    int lanesOffS = 120;
    std::string lanesSchedule;               // off:S,on:S,... (B0); overrides lanesOffS
    int stallS = 10;
    // P5b
    int ringFps = 30;                        // CSI encode rate feeding the clip ring
    int clipRingMb = -1;                     // csi 256, usb 128
    double clipBudgetGbH = 1.5;              // per camera: 3 GB/h for both (plan 12.8)
    bool clips = true;
    double randomClipMin = 10;               // 0 = no random clips
    double tensorHz = 0.2;                   // periodic dumps; 0 = off
    bool eventDumps = true;
    int statsHz = 2;
    bool mount = true;
    MountRoi mountRoi;
    bool mountRoiSet = false;
    std::string mountRef;                    // DIR with ref.pgm (+ roi.txt); empty = parked lead-in
    int mountRefS = 10;
    int stills = 0;                          // calibration stills (1 per second from the ring)
    int instance = 0;                        // > 0: restarted by the supervisor (append, part base)
};

[[noreturn]] void usage(const char* msg = nullptr) {
    if (msg) std::cerr << "lane_drive: " << msg << "\n";
    std::cerr <<
        "usage: lane_drive --cam csi|usb --out RUN_DIR [--engine E] [--crop T,B] [--refy Y]\n"
        "                  [--hz 20] [--branch-fps 20] [--rate-gate pts|legacy] [--review-fps N]\n"
        "                  [--jpeg-q 75] [--usb-dev DEV] [--usb-decode hw|sw]\n"
        "                  [--usb-exposure framerate|camera] [--target-luma 110] [--focus 690|-1]\n"
        "                  [--minutes N] [--lanes-off-s 120] [--lanes-schedule off:S,on:S,...] [--stall-s 10]\n"
        "                  [--ring-fps 30] [--clip-ring-mb N] [--clip-budget-gb-h 1.5] [--no-clips]\n"
        "                  [--random-clip-min 10] [--tensor-hz 0.2] [--no-event-dumps] [--stats-hz 2]\n"
        "                  [--no-mount] [--mount-roi y0,y1,x0,x1] [--mount-ref DIR] [--mount-ref-s 10]\n"
        "                  [--stills N] [--instance K]\n"
        "       lane_drive --mount-replay --out DIR --files A.mkv,B.mkv [--ref-file F --ref-at S (into F)]\n"
        "                  [--ref-n 9] [--mount-roi y0,y1,x0,x1] [--tap WxH]\n";
    std::exit(1);
}

std::vector<float> parseFloats(const std::string& v) {
    std::vector<float> out;
    std::stringstream ss(v);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(std::stof(item));
    return out;
}

bool parseRoi(const std::string& v, MountRoi& r) {
    const auto f = parseFloats(v);
    if (f.size() != 2 && f.size() != 4) return false;
    r.y0 = f[0]; r.y1 = f[1];
    if (f.size() == 4) { r.x0 = f[2]; r.x1 = f[3]; }
    return r.y0 >= 0 && r.y1 <= 1 && r.y0 < r.y1 && r.x0 >= 0 && r.x1 <= 1 && r.x0 < r.x1;
}

Options parseOptions(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage(("missing value for " + a).c_str());
            return argv[++i];
        };
        if      (a == "--cam")          o.cam = next();
        else if (a == "--out")          o.out = next();
        else if (a == "--engine")       o.engine = next();
        else if (a == "--crop") {
            const auto f = parseFloats(next());
            if (f.size() != 2) usage("--crop wants T,B");
            o.cropTop = f[0];
            o.cropBottom = f[1];
        }
        else if (a == "--refy")         o.refY = std::stof(next());
        else if (a == "--hz")           o.hz = static_cast<uint32_t>(std::stoul(next()));
        else if (a == "--branch-fps")   o.branchFps = static_cast<uint32_t>(std::stoul(next()));
        else if (a == "--rate-gate")    o.rateGate = next();
        else if (a == "--review-fps")   o.reviewFps = std::stoi(next());
        else if (a == "--jpeg-q")       o.jpegQ = std::stoi(next());
        else if (a == "--usb-dev")      o.usbDev = next();
        else if (a == "--usb-decode")   o.usbDecode = next();
        else if (a == "--usb-exposure") o.usbExposure = next();
        else if (a == "--target-luma")  o.targetLuma = std::stoi(next());
        else if (a == "--focus")        o.focus = std::stoi(next());
        else if (a == "--minutes")      o.minutes = std::stod(next());
        else if (a == "--lanes-off-s")  o.lanesOffS = std::stoi(next());
        else if (a == "--lanes-schedule") o.lanesSchedule = next();
        else if (a == "--stall-s")      o.stallS = std::stoi(next());
        else if (a == "--ring-fps")     o.ringFps = std::stoi(next());
        else if (a == "--clip-ring-mb") o.clipRingMb = std::stoi(next());
        else if (a == "--clip-budget-gb-h") o.clipBudgetGbH = std::stod(next());
        else if (a == "--no-clips")     o.clips = false;
        else if (a == "--random-clip-min") o.randomClipMin = std::stod(next());
        else if (a == "--tensor-hz")    o.tensorHz = std::stod(next());
        else if (a == "--no-event-dumps") o.eventDumps = false;
        else if (a == "--stats-hz")     o.statsHz = std::stoi(next());
        else if (a == "--no-mount")     o.mount = false;
        else if (a == "--mount-roi")    { if (!parseRoi(next(), o.mountRoi)) usage("--mount-roi y0,y1[,x0,x1] fractions"); o.mountRoiSet = true; }
        else if (a == "--mount-ref")    o.mountRef = next();
        else if (a == "--mount-ref-s")  o.mountRefS = std::stoi(next());
        else if (a == "--stills")       o.stills = std::stoi(next());
        else if (a == "--instance")     o.instance = std::stoi(next());
        else usage(("unknown option " + a).c_str());
    }
    if (o.cam != "csi" && o.cam != "usb") usage("--cam must be csi or usb");
    if (o.out.empty()) usage("--out is required");
    if (o.rateGate != "pts" && o.rateGate != "legacy") usage("--rate-gate pts|legacy");
    if (o.usbDecode != "hw" && o.usbDecode != "sw") usage("--usb-decode hw|sw");
    if (o.usbExposure != "framerate" && o.usbExposure != "camera") usage("--usb-exposure framerate|camera");
    if (o.statsHz < 1 || o.statsHz > 10) usage("--stats-hz 1..10");
    // Per-camera defaults.  CSI: the libconfig <Detection> crops (rows
    // [544, 864) of 1088) until the mount calibration (12.1) sets them.
    // UGREEN: top half only — the CSI bottom crop cuts the UGREEN's nearest
    // lane markings (P1 finding: 0/298 night frames with lanes).
    if (o.cropTop < 0)    o.cropTop    = 0.5f;
    if (o.cropBottom < 0) o.cropBottom = o.cam == "csi" ? 0.2059f : 0.0f;
    if (o.reviewFps < 0)  o.reviewFps  = o.cam == "csi" ? 5 : 10;
    if (o.clipRingMb < 0) o.clipRingMb = o.cam == "csi" ? 256 : 128;
    // Car-fixed region for the mount monitor until a calibration says where
    // the hood / dashboard is: the bottom quarter (CSI) / fifth (UGREEN).
    if (!o.mountRoiSet) o.mountRoi.y0 = o.cam == "csi" ? 0.75f : 0.80f;
    return o;
}

// ─── small helpers ───────────────────────────────────────────────────────────

std::string jsonStr(const std::string& s) {
    std::string r = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') { r += '\\'; r += c; }
        else if (c == '\n') r += "\\n";
        else if (static_cast<unsigned char>(c) < 0x20) r += ' ';
        else r += c;
    }
    return r + "\"";
}

long procStatusKb(const char* key) {
    std::ifstream in("/proc/self/status");
    std::string line;
    while (std::getline(in, line))
        if (line.rfind(key, 0) == 0) return std::atol(line.c_str() + std::strlen(key));
    return -1;
}

int countDir(const char* path) {
    DIR* d = ::opendir(path);
    if (!d) return -1;
    int n = 0;
    while (dirent* e = ::readdir(d)) n += e->d_name[0] != '.';
    ::closedir(d);
    return n;
}

bool isImx296(const cameraInfo& info) {
    const auto slash = info.address.find_last_of('/');
    const std::string node = slash == std::string::npos ? info.address : info.address.substr(slash + 1);
    std::ifstream f("/sys/class/video4linux/" + node + "/name");
    std::string name((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return name.find("imx296") != std::string::npos;
}

std::string readSmall(const std::string& path) {
    std::ifstream f(path);
    if (!f) return std::string();
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// Write @p text to @p path via a temporary + rename (readers never see half a file).
bool writeAtomic(const std::string& path, const std::string& text) {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::trunc);
        if (!f) return false;
        f << text;
        if (!f) return false;
    }
    return std::rename(tmp.c_str(), path.c_str()) == 0;
}

/// Top-level number / string of a flat JSON object (live_state.json, status.json).
bool jsonNumber(const std::string& j, const std::string& key, double& v) {
    const auto p = j.find("\"" + key + "\"");
    if (p == std::string::npos) return false;
    const auto c = j.find(':', p);
    if (c == std::string::npos) return false;
    const char* s = j.c_str() + c + 1;
    while (*s == ' ') ++s;
    if (std::strncmp(s, "null", 4) == 0) return false;
    char* end = nullptr;
    v = std::strtod(s, &end);
    return end != s;
}
bool jsonString(const std::string& j, const std::string& key, std::string& v) {
    const auto p = j.find("\"" + key + "\"");
    if (p == std::string::npos) return false;
    const auto q = j.find('"', j.find(':', p) + 1);
    if (q == std::string::npos) return false;
    v.clear();
    for (size_t i = q + 1; i < j.size(); ++i) {
        if (j[i] == '\\' && i + 1 < j.size()) { v += j[++i]; continue; }
        if (j[i] == '"') return true;
        v += j[i];
    }
    return false;
}

// A bin from @p desc with a "sink" ghost pad on its first element.  Not
// gst_parse_bin_from_description(ghost_unlinked=TRUE): with automatic
// ghosting, splitmuxsink fails its READY transition ("Failed to link muxer and
// sink - splitmuxsink will not work", seen on this GStreamer 1.20).
GstElement* branchBin(const std::string& desc, std::string& why) {
    GError* err = nullptr;
    GstElement* bin = gst_parse_bin_from_description(desc.c_str(), FALSE, &err);
    if (!bin || err) {
        why = std::string("branch bin: ") + (err ? err->message : "?");
        if (err) g_error_free(err);
        if (bin) gst_object_unref(bin);
        return nullptr;
    }
    GstElement* first = nullptr;
    GstIterator* it = gst_bin_iterate_sorted(GST_BIN(bin));   // sinks first: the last one is upstream-most
    GValue v = G_VALUE_INIT;
    while (gst_iterator_next(it, &v) == GST_ITERATOR_OK) {
        if (first) gst_object_unref(first);
        first = GST_ELEMENT(g_value_dup_object(&v));
        g_value_reset(&v);
    }
    g_value_unset(&v);
    gst_iterator_free(it);
    GstPad* target = first ? gst_element_get_static_pad(first, "sink") : nullptr;
    const bool ok = target && gst_element_add_pad(bin, gst_ghost_pad_new("sink", target));
    if (target) gst_object_unref(target);
    if (first) gst_object_unref(first);
    if (!ok) {
        why = "branch bin: no sink pad to ghost";
        gst_object_unref(bin);
        return nullptr;
    }
    return bin;
}

// Sinks inside @p bin stop waiting for preroll (async=false): a lane branch
// behind a closed valve must not hold a live pipeline in PAUSED (the review
// sink would then never write) — what Camera_GST does for initialEnabled=false.
void sinksNotAsync(GstElement* bin) {
    GstIterator* it = gst_bin_iterate_sinks(GST_BIN(bin));
    GValue v = G_VALUE_INIT;
    while (gst_iterator_next(it, &v) == GST_ITERATOR_OK) {
        GObject* e = static_cast<GObject*>(g_value_get_object(&v));
        if (g_object_class_find_property(G_OBJECT_GET_CLASS(e), "async"))
            g_object_set(e, "async", FALSE, nullptr);
        g_value_reset(&v);
    }
    g_value_unset(&v);
    gst_iterator_free(it);
}

// The UGREEN's own (driver) buffer timestamps run ~25 % fast against the
// system clock (measured 2026-10-25: PTS-based 20 fps decimation passed 25
// frames per wall second and capture->done came out at -1.5..-16 s).  Restamp
// every buffer with its arrival time on the pipeline clock (running time):
// decimation, the PTS rate gate, captureMonoNs and the review MKV timing then
// all follow the real clock.
GstPadProbeReturn restampProbe(GstPad*, GstPadProbeInfo* info, gpointer user) {
    GstElement* pipe = static_cast<GstElement*>(user);
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    GstClock* clk = gst_element_get_clock(pipe);
    if (!buf || !clk) { if (clk) gst_object_unref(clk); return GST_PAD_PROBE_OK; }
    const GstClockTime now = gst_clock_get_time(clk);
    gst_object_unref(clk);
    const GstClockTime base = gst_element_get_base_time(pipe);
    if (now < base) return GST_PAD_PROBE_OK;
    buf = gst_buffer_make_writable(buf);
    GST_BUFFER_PTS(buf) = now - base;
    GST_BUFFER_DTS(buf) = GST_CLOCK_TIME_NONE;
    GST_PAD_PROBE_INFO_DATA(info) = buf;
    return GST_PAD_PROBE_OK;
}

// Drop-only decimation to a rate, on the (restamped) arrival PTS, with half a
// source frame interval of slack — the library's RateGate::kPts rule.  Used
// instead of videorate on the UGREEN lane and review paths: videorate stamps
// output on a timeline of its own, which on this live pipeline ran ~1 s ahead
// of the source after start-up (capture->done measured at -0.9 s; 22-25 Hz for 20).
// interval may change at run time (the GPU-saturation step lowers the lane rate).
struct Decimator {
    std::atomic<uint64_t> interval{0};
    GstClockTime next = GST_CLOCK_TIME_NONE;
    GstClockTime lastPts = GST_CLOCK_TIME_NONE;
    void reset(uint64_t iv) { interval = iv; next = GST_CLOCK_TIME_NONE; lastPts = GST_CLOCK_TIME_NONE; }
};
GstPadProbeReturn decimateProbe(GstPad*, GstPadProbeInfo* info, gpointer user) {
    auto* d = static_cast<Decimator*>(user);
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    const GstClockTime iv = d->interval.load(std::memory_order_relaxed);
    if (!buf || iv == 0) return GST_PAD_PROBE_OK;
    const GstClockTime pts = GST_BUFFER_PTS(buf);
    if (!GST_CLOCK_TIME_IS_VALID(pts)) return GST_PAD_PROBE_OK;
    const GstClockTime frame = GST_CLOCK_TIME_IS_VALID(d->lastPts) && pts > d->lastPts ? pts - d->lastPts : 0;
    d->lastPts = pts;
    if (!GST_CLOCK_TIME_IS_VALID(d->next) || pts + iv < d->next || pts > d->next + iv) {
        d->next = pts + iv;
        return GST_PAD_PROBE_OK;
    }
    if (pts + frame / 2 > d->next) {
        d->next += iv;
        return GST_PAD_PROBE_OK;
    }
    return GST_PAD_PROBE_DROP;
}

void addDecimator(GstElement* bin, const char* name, Decimator* d) {
    GstElement* el = gst_bin_get_by_name(GST_BIN(bin), name);
    if (!el) return;
    if (GstPad* p = gst_element_get_static_pad(el, "src")) {
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, decimateProbe, d, nullptr);
        gst_object_unref(p);
    }
    gst_object_unref(el);
}

// Counts buffers through a pad (the tee's sink = frames the source offered).
GstPadProbeReturn countProbe(GstPad*, GstPadProbeInfo*, gpointer user) {
    static_cast<std::atomic<uint64_t>*>(user)->fetch_add(1, std::memory_order_relaxed);
    return GST_PAD_PROBE_OK;
}

void addCountProbe(GstElement* el, const char* pad, std::atomic<uint64_t>* counter) {
    if (!el) return;
    if (GstPad* p = gst_element_get_static_pad(el, pad)) {
        gst_pad_add_probe(p, GST_PAD_PROBE_TYPE_BUFFER, countProbe, counter, nullptr);
        gst_object_unref(p);
    }
}

// PTS → CLOCK_MONOTONIC through @p el's pipeline clock (the library's rule:
// only when that clock is the monotonic system clock; else −1).
int64_t captureMono(GstElement* el, GstSample* sample, GstBuffer* buf) {
    const GstClockTime pts = GST_BUFFER_PTS(buf);
    const GstSegment* seg = gst_sample_get_segment(sample);
    if (!GST_CLOCK_TIME_IS_VALID(pts) || !seg || seg->format != GST_FORMAT_TIME) return -1;
    const GstClockTime rt = gst_segment_to_running_time(seg, GST_FORMAT_TIME, pts);
    if (!GST_CLOCK_TIME_IS_VALID(rt)) return -1;
    GstClock* clk = gst_element_get_clock(el);
    if (!clk) return -1;
    bool mono = false;
    if (GST_IS_SYSTEM_CLOCK(clk)) {
        GstClockType type = GST_CLOCK_TYPE_REALTIME;
        g_object_get(clk, "clock-type", &type, nullptr);
        mono = type == GST_CLOCK_TYPE_MONOTONIC;
    }
    gst_object_unref(clk);
    return mono ? static_cast<int64_t>(gst_element_get_base_time(el) + rt) : -1;
}

// Clip ring feeder: every encoded frame the ring appsink receives is copied
// into the JpegRing (streaming thread).
struct RingTap {
    JpegRing* ring = nullptr;
    std::atomic<uint64_t> frames{0};
    static GstFlowReturn onSample(GstAppSink* sink, gpointer user) {
        auto* t = static_cast<RingTap*>(user);
        GstSample* s = gst_app_sink_pull_sample(sink);
        if (!s) return GST_FLOW_OK;
        GstBuffer* b = gst_sample_get_buffer(s);
        GstMapInfo m;
        if (b && t->ring && gst_buffer_map(b, &m, GST_MAP_READ)) {
            const int64_t pts = GST_BUFFER_PTS_IS_VALID(b) ? static_cast<int64_t>(GST_BUFFER_PTS(b)) : -1;
            int64_t mono = captureMono(GST_ELEMENT(sink), s, b);
            if (mono < 0) mono = monoNs();
            t->ring->push(pts, mono, m.data, m.size);
            gst_buffer_unmap(b, &m);
            t->frames.fetch_add(1, std::memory_order_relaxed);
        }
        gst_sample_unref(s);
        return GST_FLOW_OK;
    }
    void attach(GstElement* appsink) {
        GstAppSinkCallbacks cb{};
        cb.new_sample = &RingTap::onSample;
        gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &cb, this, nullptr);
    }
};

/// The Y plane (or GRAY8 frame) of @p s, block-averaged by @p f.
bool grayFromSample(GstSample* s, int f, Gray& g) {
    GstVideoInfo info;
    GstVideoFrame vf;
    if (!gst_video_info_from_caps(&info, gst_sample_get_caps(s))) return false;
    if (!(GST_VIDEO_INFO_IS_YUV(&info) || GST_VIDEO_INFO_FORMAT(&info) == GST_VIDEO_FORMAT_GRAY8)) return false;
    if (!gst_video_frame_map(&vf, &info, gst_sample_get_buffer(s), GST_MAP_READ)) return false;
    const auto* y = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&vf, 0));
    const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vf, 0);
    const int w = GST_VIDEO_FRAME_WIDTH(&vf), h = GST_VIDEO_FRAME_HEIGHT(&vf);
    f = std::max(1, f);
    g.w = w / f;
    g.h = h / f;
    g.px.assign(static_cast<size_t>(g.w) * g.h, 0);
    for (int r = 0; r < g.h; ++r)
        for (int c = 0; c < g.w; ++c) {
            uint32_t sum = 0;
            for (int dy = 0; dy < f; ++dy) {
                const uint8_t* row = y + static_cast<size_t>(r * f + dy) * stride + c * f;
                for (int dx = 0; dx < f; ++dx) sum += row[dx];
            }
            g.px[static_cast<size_t>(r) * g.w + c] = static_cast<uint8_t>(sum / (f * f));
        }
    gst_video_frame_unmap(&vf);
    return true;
}

// ─── events ──────────────────────────────────────────────────────────────────

class Events {
public:
    bool open(const std::string& path, const std::string& cam, bool append) {
        cam_ = cam;
        return w_.open(path, "mono_ns,realtime_ns,cam,type,value,detail", append);
    }
    void add(const std::string& type, const std::string& value = "", const std::string& detail = "") {
        std::string d = detail;
        for (char& c : d) if (c == ',' || c == '\n') c = ';';
        w_.row(std::to_string(monoNs()) + "," + std::to_string(realNs()) + "," + cam_ + "," + type + "," +
               value + "," + d);
        std::fprintf(stderr, "[lane_drive %s] %s %s %s\n", cam_.c_str(), type.c_str(), value.c_str(), d.c_str());
        std::lock_guard<std::mutex> lk(m_);
        last_ = type + (value.empty() ? "" : " " + value);
    }
    std::string last() const { std::lock_guard<std::mutex> lk(m_); return last_; }
    CsvWriter& writer() { return w_; }
private:
    std::string cam_;
    CsvWriter w_;
    mutable std::mutex m_;
    std::string last_;
};

// ─── camera rigs ─────────────────────────────────────────────────────────────

class Rig {
public:
    virtual ~Rig() = default;
    /// Build and start the camera with @p laneBin (floating, ownership taken).
    virtual bool start(GstElement* laneBin, bool lanesOn, int part, std::string& why) = 0;
    virtual void stop() = 0;                          ///< EOS + teardown (MKVs finalised)
    virtual bool failed(std::string& why) = 0;        ///< camera/pipeline error since start
    virtual void setLanes(bool on) = 0;
    virtual void setReview(bool on) = 0;
    virtual void setLaneHz(int /*hz*/) {}
    virtual void close() {}                           ///< release the device for good
    virtual double nominalFps() const = 0;
    virtual int ringW() const = 0;
    virtual int ringH() const = 0;
    virtual double ringFps() const = 0;
    /// The newest stats-tap frame (non-blocking), as a small grey image.
    virtual bool statsFrame(Gray& g, int64_t& pts) = 0;
    /// Stats image px per source px (tap width / source width).
    virtual float statsScale() const = 0;
    uint64_t offered() const { return offered_.load(std::memory_order_relaxed); }
    uint64_t ringFrames() const { return ringTap_.frames.load(std::memory_order_relaxed); }
protected:
    bool pullGray(GstElement* sink, int f, Gray& g, int64_t& pts) {
        if (!sink) return false;
        GstSample* s = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 0);
        if (!s) return false;
        GstBuffer* b = gst_sample_get_buffer(s);
        pts = b && GST_BUFFER_PTS_IS_VALID(b) ? static_cast<int64_t>(GST_BUFFER_PTS(b)) : -1;
        const bool ok = grayFromSample(s, f, g);
        gst_sample_unref(s);
        return ok;
    }
    std::atomic<uint64_t> offered_{0};
    RingTap ringTap_;
};

constexpr int kCsiTapW = 364, kCsiTapH = 272;        // 1456x1088 / 4 (VIC)

class CsiRig : public Rig {
public:
    CsiRig(const Options& o, const std::string& reviewDir, JpegRing* ring, dashcam::log::LogCallback log)
        : o_(o), reviewDir_(reviewDir), log_(std::move(log)) { ringTap_.ring = ring; }
    ~CsiRig() override { release(); }
    bool init(std::string& why, uint32_t& w, uint32_t& h) {
        std::vector<cameraInfo> cams;
        if (getCameraList(cams) != ERROR_CODE::NONE) { why = "getCameraList failed"; return false; }
        for (const auto& c : cams)
            if (c.type == CAMERA_TYPE::CSI && isImx296(c) && !c.videoFormats.empty()) { info_ = c; found_ = true; break; }
        if (!found_) { why = "no IMX296 CSI camera"; return false; }
        w = info_.videoFormats[0].width;
        h = info_.videoFormats[0].height;
        w_ = static_cast<int>(w);
        h_ = static_cast<int>(h);
        fps_ = info_.videoFormats[0].frameRate;
        cam_ = std::make_unique<Camera_CSI>(info_);
        cam_->setLogCallback(log_);
        cam_->open();
        return true;
    }
    bool start(GstElement* laneBin, bool lanesOn, int part, std::string& why) override {
        release();
        // One 30 fps NVJPG encode feeds the review muxer (decimated to
        // --review-fps; a valve for the degrade ladder) and the clip ring.
        const std::string encode =
            "videorate drop-only=true max-rate=" + std::to_string(o_.ringFps) +
            " ! nvjpegenc quality=" + std::to_string(o_.jpegQ) + " ! tee name=et "
            "et. ! queue leaky=downstream max-size-buffers=8 max-size-bytes=0 max-size-time=0 "
            "! valve name=revvalve drop=false ! identity name=revdecim "
            "! splitmuxsink muxer-factory=matroskamux max-size-time=60000000000 location=" +
            reviewDir_ + "/csi_p" + std::to_string(part) + "_%05d.mkv "
            "et. ! queue leaky=downstream max-size-buffers=8 max-size-bytes=0 max-size-time=0 "
            "! appsink name=ring sync=false async=false max-buffers=8 drop=true";
        const std::string stats =
            "videorate drop-only=true max-rate=" + std::to_string(o_.statsHz) +
            " ! nvvidconv ! video/x-raw,format=GRAY8,width=" + std::to_string(kCsiTapW) + ",height=" +
            std::to_string(kCsiTapH) + " ! appsink name=stats sync=false async=false max-buffers=1 drop=true";
        GstElement* ebin = branchBin(encode, why);
        GstElement* sbin = ebin ? branchBin(stats, why) : nullptr;
        if (!ebin || !sbin) {
            if (laneBin) gst_object_unref(gst_object_ref_sink(laneBin));
            if (ebin) gst_object_unref(ebin);
            return false;
        }
        // Review decimation by probe, not a second videorate: videorate's
        // caps (max-rate 5) negotiated the upstream 30 fps videorate down to
        // ~5 fps, starving the clip ring (measured 2026-10-26: 392 ring frames
        // in 75 s).
        revDecim_.reset(o_.reviewFps > 0 ? GST_SECOND / static_cast<uint64_t>(o_.reviewFps) : 0);
        addDecimator(ebin, "revdecim", &revDecim_);
        revValve_ = gst_bin_get_by_name(GST_BIN(ebin), "revvalve");
        ringSink_ = gst_bin_get_by_name(GST_BIN(ebin), "ring");
        statsSink_ = gst_bin_get_by_name(GST_BIN(sbin), "stats");
        if (ringSink_) ringTap_.attach(ringSink_);
        cam_->addBranch("lanes", laneBin, /*leaky=*/true, lanesOn);
        cam_->addBranch("encode", ebin, /*leaky=*/true, true);
        cam_->addBranch("stats", sbin, /*leaky=*/true, true);
        cam_->setCameraVideoFormat(0);
        cam_->start();
        cameraStatus st;
        cam_->getCameraStatus(st);
        if (st.status != CAMERA_STATUS::RUNNING) { why = "Camera_CSI did not reach RUNNING"; return false; }
        cam_->setCaptureEnabled(false);              // its CPU BGR conversion is not ours
        addCountProbe(cam_->getTee(), "sink", &offered_);
        if (!reviewOn_) setReview(false);
        return true;
    }
    void stop() override {
        if (!cam_) return;
        if (revValve_) g_object_set(revValve_, "drop", FALSE, nullptr);   // a closed valve swallows the EOS
        cam_->stop();
        release();
    }
    void close() override { if (cam_) cam_->close(); }
    bool failed(std::string& why) override {
        cameraStatus st;
        cam_->getCameraStatus(st);
        if (st.status == CAMERA_STATUS::ERROR) { why = "Camera_CSI status ERROR"; return true; }
        return false;
    }
    void setLanes(bool on) override { cam_->setBranchEnabled("lanes", on); }
    void setReview(bool on) override {
        reviewOn_ = on;
        if (revValve_) g_object_set(revValve_, "drop", on ? FALSE : TRUE, nullptr);
    }
    double nominalFps() const override { return fps_; }
    int ringW() const override { return w_; }
    int ringH() const override { return h_; }
    double ringFps() const override { return o_.ringFps; }
    bool statsFrame(Gray& g, int64_t& pts) override { return pullGray(statsSink_, 1, g, pts); }
    float statsScale() const override { return static_cast<float>(kCsiTapW) / std::max(1, w_); }
private:
    void release() {
        if (revValve_)  { gst_object_unref(revValve_);  revValve_ = nullptr; }
        if (ringSink_)  { gst_object_unref(ringSink_);  ringSink_ = nullptr; }
        if (statsSink_) { gst_object_unref(statsSink_); statsSink_ = nullptr; }
    }
    Options o_;
    std::string reviewDir_;
    dashcam::log::LogCallback log_;
    cameraInfo info_{};
    bool found_ = false;
    double fps_ = 60;
    int w_ = 1456, h_ = 1088;
    bool reviewOn_ = true;
    std::unique_ptr<Camera_CSI> cam_;
    GstElement* revValve_ = nullptr;
    GstElement* ringSink_ = nullptr;
    GstElement* statsSink_ = nullptr;
    Decimator revDecim_;
};

class UsbRig : public Rig {
public:
    UsbRig(const Options& o, const std::string& reviewDir, JpegRing* ring) : o_(o), reviewDir_(reviewDir) {
        ringTap_.ring = ring;
    }
    ~UsbRig() override { stop(); }
    bool start(GstElement* laneBin, bool lanesOn, int part, std::string& why) override {
        const std::string decode = o_.usbDecode == "hw"
            ? "jpegparse ! nvv4l2decoder mjpeg=1 enable-max-performance=true disable-dpb=true" : "jpegdec";
        const std::string q8 = "queue leaky=downstream max-size-buffers=8 max-size-bytes=0 max-size-time=0";
        const std::string q2 = "queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0";
        const std::string desc =
            "v4l2src name=src device=" + o_.usbDev + " ! image/jpeg,width=" + std::to_string(o_.usbW) +
            ",height=" + std::to_string(o_.usbH) + ",framerate=" + std::to_string(o_.usbFps) + "/1 ! tee name=t "
            "t. ! " + q8 + " ! valve name=revvalve drop=" + std::string(reviewOn_ ? "false" : "true") +
            " ! identity name=revdecim ! splitmuxsink muxer-factory=matroskamux max-size-time=60000000000 location=" +
            reviewDir_ + "/usb_p" + std::to_string(part) + "_%05d.mkv "
            "t. ! " + q8 + " ! appsink name=ring sync=false async=false max-buffers=8 drop=true "
            "t. ! " + q2 + " ! valve name=lanevalve drop=" + std::string(lanesOn ? "false" : "true") +
            " ! identity name=decim ! " + decode + " ! identity name=lanetail "
            "t. ! " + q2 + " ! videorate drop-only=true max-rate=" + std::to_string(o_.statsHz) + " ! jpegdec "
            "! appsink name=luma max-buffers=1 drop=true sync=false";
        GError* err = nullptr;
        pipe_ = gst_parse_launch(desc.c_str(), &err);
        if (!pipe_ || err) {
            why = std::string("usb pipeline: ") + (err ? err->message : "?");
            if (err) g_error_free(err);
            if (laneBin) gst_object_unref(gst_object_ref_sink(laneBin));
            if (pipe_) { gst_object_unref(pipe_); pipe_ = nullptr; }
            return false;
        }
        GstElement* tail = gst_bin_get_by_name(GST_BIN(pipe_), "lanetail");
        sinksNotAsync(laneBin);
        gst_bin_add(GST_BIN(pipe_), laneBin);
        const bool linked = gst_element_link(tail, laneBin);
        gst_object_unref(tail);
        if (!linked) { why = "lane bin did not link after the decoder"; teardown(); return false; }
        valve_    = gst_bin_get_by_name(GST_BIN(pipe_), "lanevalve");
        revValve_ = gst_bin_get_by_name(GST_BIN(pipe_), "revvalve");
        luma_     = gst_bin_get_by_name(GST_BIN(pipe_), "luma");
        ringSink_ = gst_bin_get_by_name(GST_BIN(pipe_), "ring");
        if (ringSink_) ringTap_.attach(ringSink_);
        GstElement* tee = gst_bin_get_by_name(GST_BIN(pipe_), "t");
        addCountProbe(tee, "sink", &offered_);
        gst_object_unref(tee);
        decim_.reset(laneHz_ > 0 ? GST_SECOND / static_cast<uint64_t>(laneHz_) : 0);
        revDecim_.reset(o_.reviewFps > 0 ? GST_SECOND / static_cast<uint64_t>(o_.reviewFps) : 0);
        addDecimator(pipe_, "decim", &decim_);
        addDecimator(pipe_, "revdecim", &revDecim_);
        GstElement* src = gst_bin_get_by_name(GST_BIN(pipe_), "src");
        if (GstPad* sp = gst_element_get_static_pad(src, "src")) {
            gst_pad_add_probe(sp, GST_PAD_PROBE_TYPE_BUFFER, restampProbe, pipe_, nullptr);
            gst_object_unref(sp);
        }
        gst_object_unref(src);
        if (gst_element_set_state(pipe_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            why = "usb pipeline refused PLAYING";
            teardown();
            return false;
        }
        return true;
    }
    void stop() override {
        if (!pipe_) return;
        // Closed valves would swallow the EOS.
        if (valve_) g_object_set(valve_, "drop", FALSE, nullptr);
        if (revValve_) g_object_set(revValve_, "drop", FALSE, nullptr);
        gst_element_send_event(pipe_, gst_event_new_eos());
        GstBus* bus = gst_element_get_bus(pipe_);
        if (GstMessage* m = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
                static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR)))
            gst_message_unref(m);
        gst_object_unref(bus);
        teardown();
    }
    bool failed(std::string& why) override {
        if (!pipe_) { why = "no pipeline"; return true; }
        GstBus* bus = gst_element_get_bus(pipe_);
        bool bad = false;
        while (GstMessage* m = gst_bus_pop_filtered(bus,
                   static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS))) {
            if (GST_MESSAGE_TYPE(m) == GST_MESSAGE_ERROR) {
                GError* e = nullptr;
                gst_message_parse_error(m, &e, nullptr);
                why = std::string("bus error: ") + (e ? e->message : "?");
                if (e) g_error_free(e);
            } else {
                why = "unexpected EOS (camera unplugged?)";
            }
            bad = true;
            gst_message_unref(m);
        }
        gst_object_unref(bus);
        return bad;
    }
    void setLanes(bool on) override { if (valve_) g_object_set(valve_, "drop", on ? FALSE : TRUE, nullptr); }
    void setReview(bool on) override {
        reviewOn_ = on;
        if (revValve_) g_object_set(revValve_, "drop", on ? FALSE : TRUE, nullptr);
    }
    void setLaneHz(int hz) override {
        laneHz_ = hz;
        decim_.interval = hz > 0 ? GST_SECOND / static_cast<uint64_t>(hz) : 0;
    }
    double nominalFps() const override { return o_.usbFps; }
    int ringW() const override { return o_.usbW; }
    int ringH() const override { return o_.usbH; }
    double ringFps() const override { return o_.usbFps; }
    bool statsFrame(Gray& g, int64_t& pts) override { return pullGray(luma_, 4, g, pts); }
    float statsScale() const override { return 0.25f; }
private:
    void teardown() {
        if (!pipe_) return;
        gst_element_set_state(pipe_, GST_STATE_NULL);
        for (GstElement** e : { &valve_, &revValve_, &luma_, &ringSink_ })
            if (*e) { gst_object_unref(*e); *e = nullptr; }
        gst_object_unref(pipe_);
        pipe_ = nullptr;
    }
    Options o_;
    std::string reviewDir_;
    GstElement* pipe_     = nullptr;
    GstElement* valve_    = nullptr;
    GstElement* revValve_ = nullptr;
    GstElement* luma_     = nullptr;
    GstElement* ringSink_ = nullptr;
    Decimator   decim_, revDecim_;
    int         laneHz_ = 20;
    bool        reviewOn_ = true;
public:
    void initLaneHz(int hz) { laneHz_ = hz; }
};

// V4L2 control reads for camera_ctrl.csv (a second fd; reads are allowed
// while another fd streams).
int readCtrl(int fd, uint32_t id) {
    v4l2_control c{};
    c.id = id;
    return fd >= 0 && ioctl(fd, VIDIOC_G_CTRL, &c) == 0 ? c.value : INT32_MIN;
}
std::string ctrlStr(int v) { return v == INT32_MIN ? std::string() : std::to_string(v); }

// ─── frames.csv ──────────────────────────────────────────────────────────────

std::string framesHeader() {
    std::string h = "cam,seq,pts_ns,capture_mono_ns,pull_mono_ns,done_mono_ns,lat_cap_done_ms,lat_pull_done_ms,"
                    "h2d_us,pre_us,trt_us,d2h_us,dec_us,valid,num_lanes,ego,lat_off,lat_valid,nonfinite";
    for (int k = 0; k < 4; ++k)
        for (const char* f : { "det", "head", "npts", "slope", "icpt", "xref", "ymin", "ymax", "rms" })
            h += ",s" + std::to_string(k) + "_" + f;
    return h + ",band_luma,slot_order_ok,dump_id,live_speed_kmh,live_gear,live_stalk";
}

/// Detected slots ordered left to right at the reference row (xRef rising).
bool slotOrderOk(const LaneResult& r) {
    float last = -INFINITY;
    for (const auto& s : r.slots) {
        if (!s.detected) continue;
        if (!(s.xRef >= last)) return false;
        last = s.xRef;
    }
    return true;
}

/// Live CAN state as the cameras see it (main loop writes, sink reads).
struct LiveCan {
    std::atomic<float> speed{NAN};
    std::atomic<int> gear{0};            // a letter, 0 = unknown
    std::atomic<int> stalk{0};           // −1 left, +1 right, 0 none
    std::atomic<int64_t> ageMs{-1};
    std::atomic<bool> have{false};
};

std::string frameRow(const std::string& cam, const LaneResult& r, float bandLuma, int dumpId, const LiveCan& live) {
    const LaneTiming& t = r.timing;
    std::string s;
    s.reserve(640);
    auto ns = [](int64_t v) { return v >= 0 ? std::to_string(v) : std::string(); };
    auto us = [](float v) { return v >= 0 ? fmtF(v) : std::string(); };
    s += cam + "," + std::to_string(r.sequence) + "," + ns(t.ptsNs) + "," + ns(t.captureMonoNs) + "," +
         ns(t.pullMonoNs) + "," + ns(t.doneMonoNs) + ",";
    s += (t.captureMonoNs >= 0 && t.doneMonoNs >= 0 ? fmtF((t.doneMonoNs - t.captureMonoNs) / 1e6) : "") + ",";
    s += (t.pullMonoNs >= 0 && t.doneMonoNs >= 0 ? fmtF((t.doneMonoNs - t.pullMonoNs) / 1e6) : "") + ",";
    s += us(t.h2dUs) + "," + us(t.preUs) + "," + us(t.trtUs) + "," + us(t.d2hUs) + "," + us(t.decUs) + ",";
    s += std::to_string(r.valid) + "," + std::to_string(r.numLanes) + "," + std::to_string(r.currentLaneIndex) + "," +
         (r.lateralValid ? fmtF(r.lateralOffset) : "") + "," + std::to_string(r.lateralValid) + "," +
         std::to_string(r.nonFinite);
    for (const auto& sl : r.slots) {
        s += "," + std::to_string(sl.detected) + "," + std::to_string(static_cast<int>(sl.head)) + "," +
             std::to_string(sl.nPts);
        if (sl.detected)
            s += "," + fmtF(sl.slope) + "," + fmtF(sl.intercept) + "," + fmtF(sl.xRef) + "," + fmtF(sl.yMin) +
                 "," + fmtF(sl.yMax) + "," + fmtF(sl.rmsPx);
        else
            s += ",,,,,,";
    }
    s += "," + fmtF(bandLuma) + "," + std::to_string(slotOrderOk(r)) + "," + (dumpId > 0 ? std::to_string(dumpId) : "");
    if (live.have.load()) {
        const int g = live.gear.load();
        s += "," + fmtF(live.speed.load()) + "," + (g > 0 ? std::string(1, static_cast<char>(g)) : "") + "," +
             std::to_string(live.stalk.load());
    } else {
        s += ",,,";
    }
    return s;
}

// ─── clip writer ─────────────────────────────────────────────────────────────

/// Writes finished clip jobs as MJPEG MKVs (appsrc → matroskamux) on its own
/// thread, so the main loop never waits for a disk.
class ClipWriter {
public:
    ClipWriter(std::string dir, std::string cam, CsvWriter& index, int w, int h, double fps)
        : dir_(std::move(dir)), cam_(std::move(cam)), index_(index), w_(w), h_(h), fps_(fps) {
        th_ = std::thread(&ClipWriter::loop, this);
    }
    ~ClipWriter() { finish(); }
    /// False when the queue is full (the job is dropped and counted).
    bool enqueue(ClipJob&& j) {
        {
            std::lock_guard<std::mutex> lk(m_);
            if (q_.size() >= 4) { ++dropped_; return false; }
            q_.push_back(std::move(j));
        }
        cv_.notify_one();
        return true;
    }
    void finish() {
        {
            std::lock_guard<std::mutex> lk(m_);
            if (!run_) return;
            run_ = false;
        }
        cv_.notify_one();
        if (th_.joinable()) th_.join();
    }
    uint64_t written() const { return written_.load(); }
    uint64_t dropped() const { return dropped_.load(); }
    uint64_t bytes() const { return bytes_.load(); }
private:
    void loop() {
        for (;;) {
            ClipJob j;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [this] { return !q_.empty() || !run_; });
                if (q_.empty()) return;
                j = std::move(q_.front());
                q_.pop_front();
            }
            write(j);
        }
    }
    void write(const ClipJob& j) {
        const int64_t t0 = monoNs();
        std::string first = j.triggers.substr(0, j.triggers.find(';'));
        for (char& c : first) if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
        char name[160];
        std::snprintf(name, sizeof(name), "%s_clip%04d_%s.mkv", cam_.c_str(), j.id, first.c_str());
        const std::string path = dir_ + "/" + name;
        bool ok = !j.frames.empty();
        std::string why = ok ? "" : "no frames";
        if (ok) {
            const std::string desc = "appsrc name=src format=time is-live=false do-timestamp=false caps=image/jpeg,width=" +
                std::to_string(w_) + ",height=" + std::to_string(h_) + ",framerate=" +
                std::to_string(static_cast<int>(std::lround(fps_))) + "/1 ! matroskamux ! filesink location=" + path;
            GError* err = nullptr;
            GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
            if (!pipe || err) {
                ok = false;
                why = err ? err->message : "parse";
                if (err) g_error_free(err);
            } else {
                GstElement* src = gst_bin_get_by_name(GST_BIN(pipe), "src");
                g_object_set(src, "max-bytes", static_cast<guint64>(64u << 20), "block", TRUE, nullptr);
                gst_element_set_state(pipe, GST_STATE_PLAYING);
                const int64_t base = j.frames.front().pts;
                for (const auto& f : j.frames) {
                    GstBuffer* b = gst_buffer_new_allocate(nullptr, f.data->size(), nullptr);
                    gst_buffer_fill(b, 0, f.data->data(), f.data->size());
                    GST_BUFFER_PTS(b) = static_cast<GstClockTime>(std::max<int64_t>(0, f.pts - base));
                    if (gst_app_src_push_buffer(GST_APP_SRC(src), b) != GST_FLOW_OK) { ok = false; why = "push"; break; }
                }
                gst_app_src_end_of_stream(GST_APP_SRC(src));
                GstBus* bus = gst_element_get_bus(pipe);
                GstMessage* m = gst_bus_timed_pop_filtered(bus, 20 * GST_SECOND,
                    static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
                if (!m || GST_MESSAGE_TYPE(m) != GST_MESSAGE_EOS) { ok = false; why = m ? "error" : "eos timeout"; }
                if (m) gst_message_unref(m);
                gst_object_unref(bus);
                gst_element_set_state(pipe, GST_STATE_NULL);
                gst_object_unref(src);
                gst_object_unref(pipe);
            }
        }
        if (ok) { ++written_; bytes_ += j.bytes; }
        index_.row(std::to_string(j.id) + "," + j.triggers + "," + std::to_string(j.priority) + "," +
                   std::to_string(j.eventMono) + "," + std::to_string(j.startMono) + "," + std::to_string(j.endMono) +
                   "," + (j.frames.empty() ? "" : std::to_string(j.frames.front().pts)) + "," +
                   (j.frames.empty() ? "" : std::to_string(j.frames.front().mono)) + "," +
                   std::to_string(j.frames.size()) + "," + std::to_string(j.bytes) + "," + std::to_string(j.overflow) +
                   "," + (ok ? std::string(name) : "") + "," + fmtF((monoNs() - t0) / 1e6) + "," + why);
    }
    std::string dir_, cam_;
    CsvWriter& index_;
    int w_, h_;
    double fps_;
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<ClipJob> q_;
    bool run_ = true;
    std::thread th_;
    std::atomic<uint64_t> written_{0}, dropped_{0}, bytes_{0};
};

// ─── tensor dumps ────────────────────────────────────────────────────────────

struct DumpJob {
    int id = 0;
    std::string trigger;                 // periodic | event:<type>
    TensorSet t;
    std::vector<uint8_t> band;           // BGR, bandW × bandH (periodic only)
    int bandW = 0, bandH = 0;
};

/// File format of dumps/d<id>.bin (little-endian): "LTD1", u32 version 1,
/// u64 seq, i64 pts_ns, i64 capture_mono_ns, u32 count[4] (loc_row, loc_col,
/// exist_row, exist_col), then the float32 tensors back to back.
class DumpWriter {
public:
    DumpWriter(std::string dir, CsvWriter& index, const JpegRing& ring)
        : dir_(std::move(dir)), index_(index), ring_(ring) { th_ = std::thread(&DumpWriter::loop, this); }
    ~DumpWriter() { finish(); }
    bool enqueue(DumpJob&& j) {
        {
            std::lock_guard<std::mutex> lk(m_);
            if (q_.size() >= 16) { ++dropped_; return false; }
            q_.push_back(std::move(j));
        }
        cv_.notify_one();
        return true;
    }
    void finish() {
        {
            std::lock_guard<std::mutex> lk(m_);
            if (!run_) return;
            run_ = false;
        }
        cv_.notify_one();
        if (th_.joinable()) th_.join();
    }
    uint64_t written() const { return written_.load(); }
    uint64_t dropped() const { return dropped_.load(); }
    uint64_t bytes() const { return bytes_.load(); }
private:
    void loop() {
        for (;;) {
            DumpJob j;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [this] { return !q_.empty() || !run_; });
                if (q_.empty()) return;
                j = std::move(q_.front());
                q_.pop_front();
            }
            write(j);
        }
    }
    void write(const DumpJob& j) {
        char base[64];
        std::snprintf(base, sizeof(base), "d%06d", j.id);
        const std::string tf = std::string(base) + ".bin";
        uint64_t bytes = 0;
        if (std::FILE* f = std::fopen((dir_ + "/" + tf).c_str(), "wb")) {
            const uint32_t ver = 1;
            std::fwrite("LTD1", 1, 4, f);
            std::fwrite(&ver, 4, 1, f);
            std::fwrite(&j.t.seq, 8, 1, f);
            std::fwrite(&j.t.pts, 8, 1, f);
            std::fwrite(&j.t.captureMono, 8, 1, f);
            std::fwrite(j.t.counts, 4, 4, f);
            std::fwrite(j.t.data.data(), sizeof(float), j.t.data.size(), f);
            bytes += 44 + j.t.data.size() * sizeof(float);
            std::fclose(f);
        }
        std::string bf;
        if (!j.band.empty()) {
            const std::vector<uint8_t> enc = encodeBand(j.band, j.bandW, j.bandH);
            if (!enc.empty()) {
                bf = std::string(base) + "_band.jpg";
                if (std::FILE* f = std::fopen((dir_ + "/" + bf).c_str(), "wb")) {
                    std::fwrite(enc.data(), 1, enc.size(), f);
                    std::fclose(f);
                    bytes += enc.size();
                } else {
                    bf.clear();
                }
            }
        }
        // The camera frame with the same PTS, from the clip ring.  The JPEG
        // can still be on its way (encode vs inference): wait up to 1 s for
        // an exact match, then take the nearest.
        std::string ff;
        JFrame fr;
        int64_t dpts = 0;
        bool have = false;
        if (j.t.pts >= 0) {
            for (int i = 0; i < 20 && !(have = ring_.find(j.t.pts, fr, dpts, false)); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (!have) have = ring_.find(j.t.pts, fr, dpts, true);
        }
        if (have) {
            ff = std::string(base) + "_frame.jpg";
            if (std::FILE* f = std::fopen((dir_ + "/" + ff).c_str(), "wb")) {
                std::fwrite(fr.data->data(), 1, fr.data->size(), f);
                std::fclose(f);
                bytes += fr.data->size();
            } else {
                ff.clear();
            }
        }
        ++written_;
        bytes_ += bytes;
        index_.row(std::to_string(j.id) + "," + j.trigger + "," + std::to_string(j.t.seq) + "," +
                   std::to_string(j.t.pts) + "," + std::to_string(j.t.captureMono) + "," +
                   (have ? std::to_string(fr.pts) + "," + std::to_string(dpts) : std::string(",")) + "," + tf + "," + bf +
                   "," + ff);
    }
    // BGR band → JPEG with GStreamer's jpegenc.  Not OpenCV / libjpeg linked
    // into this binary: a libjpeg in the process's global symbol scope takes
    // nvjpegenc's jpeg_* calls ("JPEG parameter struct mismatch: library
    // thinks size is 584, caller expects 728") and kills the CSI encode branch.
    static std::vector<uint8_t> encodeBand(const std::vector<uint8_t>& bgr, int w, int h) {
        std::vector<uint8_t> out;
        const std::string desc = "appsrc name=src format=time caps=video/x-raw,format=BGR,width=" + std::to_string(w) +
            ",height=" + std::to_string(h) + ",framerate=0/1 ! videoconvert ! jpegenc quality=92 "
            "! appsink name=sink sync=false";
        GError* err = nullptr;
        GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
        if (!pipe || err) { if (err) g_error_free(err); if (pipe) gst_object_unref(pipe); return out; }
        GstElement* src = gst_bin_get_by_name(GST_BIN(pipe), "src");
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipe), "sink");
        gst_element_set_state(pipe, GST_STATE_PLAYING);
        GstBuffer* b = gst_buffer_new_allocate(nullptr, bgr.size(), nullptr);
        gst_buffer_fill(b, 0, bgr.data(), bgr.size());
        GST_BUFFER_PTS(b) = 0;
        gst_app_src_push_buffer(GST_APP_SRC(src), b);
        gst_app_src_end_of_stream(GST_APP_SRC(src));
        if (GstSample* smp = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 5 * GST_SECOND)) {
            GstMapInfo m;
            GstBuffer* ob = gst_sample_get_buffer(smp);
            if (ob && gst_buffer_map(ob, &m, GST_MAP_READ)) {
                out.assign(m.data, m.data + m.size);
                gst_buffer_unmap(ob, &m);
            }
            gst_sample_unref(smp);
        }
        gst_element_set_state(pipe, GST_STATE_NULL);
        gst_object_unref(src);
        gst_object_unref(sink);
        gst_object_unref(pipe);
        return out;
    }
    std::string dir_;
    CsvWriter& index_;
    const JpegRing& ring_;
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<DumpJob> q_;
    bool run_ = true;
    std::thread th_;
    std::atomic<uint64_t> written_{0}, dropped_{0}, bytes_{0};
};

// ─── results → main loop ─────────────────────────────────────────────────────

/// What the main loop needs from each result (filled by the sink, drained at 10 Hz).
struct ResultSig {
    int64_t mono = 0;                    // done
    float e2eMs = NAN;
    int numLanes = 0, ego = -1;
    float off = NAN;
    bool offValid = false, slotOk = true;
    int nonFinite = 0;
    uint64_t seq = 0;
};

class SigQueue {
public:
    void push(const ResultSig& s) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.size() < 4096) q_.push_back(s); else ++dropped_;
    }
    std::vector<ResultSig> drain() {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<ResultSig> out(q_.begin(), q_.end());
        q_.clear();
        return out;
    }
    uint64_t dropped() const { std::lock_guard<std::mutex> lk(m_); return dropped_; }
private:
    mutable std::mutex m_;
    std::deque<ResultSig> q_;
    uint64_t dropped_ = 0;
};

// ─── .ass banner ─────────────────────────────────────────────────────────────

/// One .ass per review part, Dialogue times on the MKV timeline (the review
/// segments keep the camera pipeline's running time: ffprobe start_time of
/// csi_p0_00004.mkv = 240.0 s), so `ffmpeg -i seg.mkv -vf ass=<cam>_p<part>.ass`
/// burns the banner into any segment of the part.
class AssWriter {
public:
    bool open(const std::string& path, int w, int h) {
        const bool fresh = !std::filesystem::exists(path);
        f_ = std::fopen(path.c_str(), "a");
        if (!f_) return false;
        if (fresh) {
            std::fprintf(f_,
                "[Script Info]\nScriptType: v4.00+\nPlayResX: %d\nPlayResY: %d\nWrapStyle: 2\n\n"
                "[V4+ Styles]\nFormat: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, "
                "BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, "
                "Shadow, Alignment, MarginL, MarginR, MarginV, Encoding\n"
                "Style: Banner,DejaVu Sans Mono,%d,&H00FFFFFF,&H000000FF,&H00000000,&H80000000,0,0,0,0,100,100,0,0,3,1,0,"
                "8,10,10,8,1\n\n[Events]\nFormat: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n",
                w, h, std::max(18, h / 30));
        }
        return true;
    }
    void line(int64_t startNs, int64_t durNs, const std::string& text) {
        if (!f_ || startNs < 0) return;
        std::fprintf(f_, "Dialogue: 0,%s,%s,Banner,,0,0,0,,%s\n", ts(startNs).c_str(), ts(startNs + durNs).c_str(),
                     text.c_str());
        if (++n_ % 25 == 0) std::fflush(f_);
    }
    void close() { if (f_) { std::fclose(f_); f_ = nullptr; } }
    ~AssWriter() { close(); }
private:
    static std::string ts(int64_t ns) {
        const int64_t cs = ns / 10'000'000;
        char b[32];
        std::snprintf(b, sizeof(b), "%lld:%02lld:%02lld.%02lld", static_cast<long long>(cs / 360000),
                      static_cast<long long>(cs / 6000 % 60), static_cast<long long>(cs / 100 % 60),
                      static_cast<long long>(cs % 100));
        return b;
    }
    std::FILE* f_ = nullptr;
    uint64_t n_ = 0;
};

// ─── PGM (mount references) ──────────────────────────────────────────────────

bool savePgm(const std::string& path, const Gray& g) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P5\n%d %d\n255\n", g.w, g.h);
    std::fwrite(g.px.data(), 1, g.px.size(), f);
    std::fclose(f);
    return true;
}

bool loadPgm(const std::string& path, Gray& g) {
    std::ifstream f(path, std::ios::binary);
    std::string magic;
    int maxv = 0;
    if (!(f >> magic >> g.w >> g.h >> maxv) || magic != "P5" || maxv != 255 || g.w <= 0 || g.h <= 0) return false;
    f.get();
    g.px.resize(static_cast<size_t>(g.w) * g.h);
    return static_cast<bool>(f.read(reinterpret_cast<char*>(g.px.data()), static_cast<std::streamsize>(g.px.size())));
}

/// Lane on/off at @p elapsedS from "off:120,on:120,off:120,on:300"; after the
/// list the last state holds.
bool scheduledLanes(const std::vector<std::pair<bool, int>>& sched, double elapsedS) {
    double t = 0;
    for (const auto& s : sched) {
        t += s.second;
        if (elapsedS < t) return s.first;
    }
    return sched.empty() ? true : sched.back().first;
}

// ─── --mount-replay (B9 on real footage) ─────────────────────────────────────

int runMountReplay(int argc, char** argv) {
    std::string out, refFile;
    std::vector<std::string> files;
    double refAt = 0;
    int refN = 9, tapW = kCsiTapW, tapH = kCsiTapH;
    MountRoi roi;
    roi.y0 = 0.75f;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage(("missing value for " + a).c_str());
            return argv[++i];
        };
        if (a == "--mount-replay") continue;
        else if (a == "--out") out = next();
        else if (a == "--files") {
            std::stringstream ss(next());
            std::string f;
            while (std::getline(ss, f, ',')) if (!f.empty()) files.push_back(f);
        }
        else if (a == "--ref-file") refFile = next();
        else if (a == "--ref-at") refAt = std::stod(next());
        else if (a == "--ref-n") refN = std::stoi(next());
        else if (a == "--mount-roi") { if (!parseRoi(next(), roi)) usage("--mount-roi y0,y1[,x0,x1]"); }
        else if (a == "--tap") { if (std::sscanf(next().c_str(), "%dx%d", &tapW, &tapH) != 2) usage("--tap WxH"); }
        else usage(("unknown option " + a).c_str());
    }
    if (out.empty() || files.empty()) usage("--mount-replay needs --out and --files");
    if (lanetest::insideRepo(out)) { std::cerr << "lane_drive: --out is inside the repo; refusing\n"; return 1; }
    std::filesystem::create_directories(out);
    if (refFile.empty()) refFile = files.front();

    // Frames of one file at 1 fps as GRAY8 tapW×tapH, with their PTS (s).
    auto decode = [&](const std::string& file, const std::function<bool(double, const Gray&)>& cb) -> bool {
        const std::string desc = "filesrc location=\"" + file + "\" ! matroskademux ! videorate drop-only=true max-rate=1 "
            "! jpegdec ! videoconvert ! videoscale ! video/x-raw,format=GRAY8,width=" + std::to_string(tapW) +
            ",height=" + std::to_string(tapH) + " ! appsink name=s sync=false max-buffers=4";
        GError* err = nullptr;
        GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
        if (!pipe || err) { if (err) g_error_free(err); if (pipe) gst_object_unref(pipe); return false; }
        GstElement* s = gst_bin_get_by_name(GST_BIN(pipe), "s");
        gst_element_set_state(pipe, GST_STATE_PLAYING);
        bool go = true;
        while (go && !g_signal) {
            GstSample* smp = gst_app_sink_try_pull_sample(GST_APP_SINK(s), 5 * GST_SECOND);
            if (!smp) break;
            Gray g;
            GstBuffer* b = gst_sample_get_buffer(smp);
            const double pts = b && GST_BUFFER_PTS_IS_VALID(b) ? GST_BUFFER_PTS(b) / 1e9 : -1;
            if (grayFromSample(smp, 1, g)) go = cb(pts, g);
            gst_sample_unref(smp);
        }
        gst_element_set_state(pipe, GST_STATE_NULL);
        gst_object_unref(s);
        gst_object_unref(pipe);
        return true;
    };

    std::vector<Gray> refs;
    double firstPts = -1;
    decode(refFile, [&](double pts, const Gray& g) {             // --ref-at: s after the file's first frame
        if (firstPts < 0) firstPts = pts;
        if (pts - firstPts >= refAt) refs.push_back(g);
        return static_cast<int>(refs.size()) < refN;
    });
    if (refs.empty()) { std::cerr << "lane_drive: no reference frames at " << refAt << " s in " << refFile << "\n"; return 2; }
    const Gray ref = medianOf(refs);
    savePgm(out + "/ref.pgm", ref);
    MountMonitor mon;
    if (!mon.setReference(ref, roi, 1456.0f / tapW)) { std::cerr << "lane_drive: bad reference / roi\n"; return 2; }
    CsvWriter csv;
    csv.open(out + "/mount_replay.csv",
             "file,pts_s,t_s,ok,dx_px,dy_px,roll_deg,conf,med5_dx,med5_dy,med5_roll,exceeding,shifted,on_edge,off_edge,luma,"
             "lost,feature_ratio");
    std::fprintf(stderr, "reference: %zu frames from %s at %.1f s, template std %.2f\n", refs.size(), refFile.c_str(),
                 refAt, mon.referenceStd());
    double base = 0;
    int onEdges = 0, samples = 0, okSamples = 0, lostSamples = 0;
    for (const auto& file : files) {
        double last = 0, first = -1;
        decode(file, [&](double pts, const Gray& g) {
            if (first < 0) first = pts;
            pts -= first;                                           // s into the file
            const double t = base + pts;
            last = pts;
            const MountSample s = mon.measure(g);
            const auto v = mon.feed(static_cast<int64_t>(t * 1e9), s);
            uint64_t sum = 0;
            for (uint8_t p : g.px) sum += p;
            ++samples;
            okSamples += s.ok;
            lostSamples += s.lost;
            onEdges += v.onEdge;
            if (v.onEdge || v.offEdge)
                std::fprintf(stderr, "%s at %s +%.0f s: med5 dx %.1f dy %.1f roll %.2f\n", v.onEdge ? "SHIFT" : "back ",
                             file.c_str(), pts, v.med5Dx, v.med5Dy, v.med5Roll);
            csv.row(std::filesystem::path(file).filename().string() + "," + fmtF(pts) + "," + fmtF(t) + "," +
                    std::to_string(s.ok) + "," + fmtF(s.dx) + "," + fmtF(s.dy) + "," + fmtF(s.rollDeg) + "," +
                    fmtF(s.conf) + "," + (v.have ? fmtF(v.med5Dx) + "," + fmtF(v.med5Dy) + "," + fmtF(v.med5Roll) : ",,") +
                    "," + std::to_string(v.exceeding) + "," + std::to_string(v.shifted) + "," +
                    std::to_string(v.onEdge) + "," + std::to_string(v.offEdge) + "," +
                    fmtF(g.px.empty() ? NAN : static_cast<double>(sum) / g.px.size()) + "," + std::to_string(s.lost) +
                    "," + fmtF(s.featureRatio));
            return true;
        });
        base += last + 1.0;
    }
    csv.close();
    std::printf("mount replay: %d samples, %d measured, %d lost, %d shift onsets -> %s/mount_replay.csv\n", samples,
                okSamples, lostSamples, onEdges, out.c_str());
    return 0;
}

} // namespace

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    gst_init(&argc, &argv);
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--mount-replay") == 0) return runMountReplay(argc, argv);
    }
    const Options o = parseOptions(argc, argv);
    namespace fs = std::filesystem;

    if (lanetest::insideRepo(o.out)) {
        std::cerr << "lane_drive: --out " << o.out << " is inside the repo; refusing\n";
        return 1;
    }
    std::vector<std::pair<bool, int>> sched;
    if (!o.lanesSchedule.empty()) {
        std::stringstream ss(o.lanesSchedule);
        std::string item;
        while (std::getline(ss, item, ',')) {
            const auto c = item.find(':');
            if (c == std::string::npos || (item.substr(0, c) != "on" && item.substr(0, c) != "off"))
                usage("--lanes-schedule wants off:S,on:S,...");
            sched.emplace_back(item.substr(0, c) == "on", std::stoi(item.substr(c + 1)));
        }
    }
    const std::string dir = o.out + "/" + o.cam;
    const std::string reviewDir = dir + "/review", clipsDir = dir + "/clips", dumpsDir = dir + "/dumps",
                      stillsDir = dir + "/stills", mountDir = dir + "/mount_ref";
    std::error_code ec;
    for (const auto& d : { reviewDir, clipsDir, dumpsDir, mountDir, o.out + "/ctl" }) {
        fs::create_directories(d, ec);
        if (ec) { std::cerr << "lane_drive: cannot create " << d << ": " << ec.message() << "\n"; return 1; }
    }
    if (o.stills > 0) fs::create_directories(stillsDir, ec);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const bool app = o.instance > 0;
    Events ev;
    CsvWriter frames(50000), counters, ctrl, clockCsv, imgStats, mountCsv, clipsIdx, dumpsIdx;
    const bool csvOk = ev.open(dir + "/events.csv", o.cam, app) &&
        frames.open(dir + "/frames.csv", framesHeader(), app) &&
        counters.open(dir + "/counters.csv",
            "mono_ns,cam_state,lanes_on,offered,pulled,gated,processed,map_fail,size_mismatch,h2d_fail,"
            "pre_fail,trt_fail,d2h_fail,sync_fail,nonfinite,eff_hz,src_fps,restarts,frames_csv_drops,"
            "rss_kb,threads,fds,ring_frames,ring_bytes,ring_span_s,clips_kept,clips_refused,clip_overflow_frames,"
            "clip_queue_drops,dumps_written,dump_queue_drops,sig_drops,degrade,lane_hz", app) &&
        ctrl.open(dir + "/camera_ctrl.csv",
            "mono_ns,exposure,exposure_unit,gain,gain_unit,auto_exposure,focus,focus_auto,src_fps,luma,"
            "night_mode,ctrl_source", app) &&
        clockCsv.open(dir + "/clock.csv", "mono_ns,realtime_ns,boottime_ns,wall_minus_mono_ns,step_ms", app) &&
        imgStats.open(dir + "/image_stats.csv",
            "mono_ns,pts_ns,band_luma_mean,band_luma_p10,band_luma_p90,sky_luma_mean,full_luma_mean,noise_sigma,"
            "motion_energy", app) &&
        mountCsv.open(dir + "/mount.csv",
            "mono_ns,dx_px,dy_px,roll_deg,method,conf,med5_dx,med5_dy,med5_roll,vp_dy,shifted,exceeding,lost,"
            "feature_ratio", app) &&
        clipsIdx.open(dir + "/clips_index.csv",
            "clip_id,triggers,priority,event_mono_ns,start_mono_ns,end_mono_ns,first_pts_ns,first_mono_ns,frames,"
            "bytes,overflow,file,write_ms,error", app) &&
        dumpsIdx.open(dir + "/dumps_index.csv",
            "dump_id,trigger,seq,pts_ns,capture_mono_ns,frame_pts_ns,dpts_ns,tensor_file,band_jpg,frame_jpg", app);
    if (!csvOk) { std::cerr << "lane_drive: cannot open CSV files in " << dir << "\n"; return 1; }

    const int64_t startMono = monoNs(), startReal = realNs();
    ev.add("start", o.cam, "instance " + std::to_string(o.instance));

    // Watchdog: a start or stop that hangs must still leave usable data.
    std::atomic<int64_t> deadlineNs{monoNs() + 90'000'000'000LL};   // start budget (engine load + Argus)
    std::atomic<const char*> stageName{"start"};
    std::atomic<bool> finished{false};
    auto writeSummary = [&](const std::string& extra) {
        // summary.json for the run's first instance, summary_i<K>.json for a
        // supervisor restart (check_run.py adds them up).
        std::ofstream s(dir + (o.instance > 0 ? "/summary_i" + std::to_string(o.instance) + ".json" : "/summary.json"));
        s << "{\"cam\": " << jsonStr(o.cam) << ", \"instance\": " << o.instance << ", \"start_mono_ns\": " << startMono
          << ", \"start_realtime_ns\": " << startReal << ", \"end_mono_ns\": " << monoNs()
          << ", \"end_realtime_ns\": " << realNs() << ", \"frames_rows\": " << frames.rows()
          << ", \"frames_csv_drops\": " << frames.drops() << extra << "}\n";
    };
    std::thread watchdog([&] {
        while (!finished) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (!finished && monoNs() > deadlineNs.load()) {
                ev.add("stop_hang", stageName.load());
                ev.writer().close();
                for (CsvWriter* w : { &frames, &counters, &ctrl, &clockCsv, &imgStats, &mountCsv, &clipsIdx, &dumpsIdx })
                    w->close();
                writeSummary(std::string(", \"stop_hang\": true, \"hung_in\": ") + jsonStr(stageName.load()));
                std::_Exit(4);
            }
        }
    });

    auto logCb = [&ev](dashcam::log::LogLevel lvl, const std::string& msg) {
        if (lvl == dashcam::log::LogLevel::ERROR || lvl == dashcam::log::LogLevel::WARN)
            ev.add(lvl == dashcam::log::LogLevel::ERROR ? "lib_error" : "lib_warn", "", msg);
    };

    // ── clip ring, camera, detector ──────────────────────────────────────────
    const bool ringWanted = o.clips || o.tensorHz > 0 || o.eventDumps || o.stills > 0;
    JpegRing ring(static_cast<size_t>(std::max(8, o.clipRingMb)) << 20);
    ring.setEnabled(ringWanted);
    Options ro = o;
    if (!ringWanted) ro.ringFps = o.reviewFps;           // no ring: encode only what review needs
    std::unique_ptr<Rig> rig;
    uint32_t srcW = 0, srcH = 0;
    std::string why;
    if (o.cam == "csi") {
        auto c = std::make_unique<CsiRig>(ro, reviewDir, &ring, logCb);
        if (!c->init(why, srcW, srcH)) { ev.add("cam_error", "init", why); finished = true; watchdog.join(); return 2; }
        rig = std::move(c);
    } else {
        srcW = static_cast<uint32_t>(o.usbW);
        srcH = static_cast<uint32_t>(o.usbH);
        auto u = std::make_unique<UsbRig>(ro, reviewDir, &ring);
        u->initLaneHz(static_cast<int>(o.branchFps));
        rig = std::move(u);
    }

    LaneDetectorConfig cfg;
    cfg.enginePath      = o.engine;
    cfg.inputCropTop    = o.cropTop;
    cfg.inputCropBottom = o.cropBottom;
    cfg.laneReferenceY  = o.refY;
    cfg.targetHz        = o.hz;
    // UGREEN: decimated before the decoder by decimateProbe, so the bin gets no
    // videorate of its own (see Decimator).
    cfg.branchMaxFps    = o.cam == "usb" ? 0 : o.branchFps;
    cfg.rateGate        = o.rateGate == "pts" ? LaneDetectorConfig::RateGate::kPts
                                              : LaneDetectorConfig::RateGate::kLegacy;
    cfg.sourceIsNVMM    = o.cam == "csi" || o.usbDecode == "hw";
    cfg.collectTimings  = true;
    cfg.threadName      = "lane_" + o.cam;
    cfg.log             = logCb;
    std::unique_ptr<LaneDetector> det;
    try {
        det = std::make_unique<LaneDetector>(srcW, srcH, cfg);
    } catch (const std::exception& e) {
        ev.add("lib_error", "construct", e.what());
        finished = true; watchdog.join();
        return 2;
    }
    const LaneGeometry geo = det->geometry();

    // ── per-result work (inference thread) ───────────────────────────────────
    // onFrame (before inference): band luma.  Sink (result published): the
    // frames.csv row, the periodic-dump decision, signals for the main loop.
    // onInference (right after, frame still mapped): tensor ring, dump copy.
    LiveCan live;
    SigQueue sigs;
    TensorRing tensors(10);
    DumpWriter dumper(dumpsDir, dumpsIdx, ring);
    std::atomic<bool> dumpsOn{o.tensorHz > 0};
    std::atomic<int> nextDumpId{o.instance * 100000};
    const int64_t dumpPeriod = o.tensorHz > 0 ? static_cast<int64_t>(1e9 / o.tensorHz) : INT64_MAX;
    float bandLuma = NAN;                         // inference thread only
    int64_t nextPeriodic = monoNs() + dumpPeriod;
    uint64_t pendingDumpSeq = 0;
    int pendingDumpId = 0;
    LaneDetectorTestHook::onFrame(*det, [&](const uint8_t* d, size_t n) {
        if (n < static_cast<size_t>(geo.branchW) * geo.branchH * 3 || geo.branchW <= 0) { bandLuma = NAN; return; }
        uint64_t sum = 0, cnt = 0;
        const size_t stride = static_cast<size_t>(geo.branchW) * 3;
        for (int y = 0; y < static_cast<int>(geo.branchH); y += 8)
            for (int x = 0; x < static_cast<int>(geo.branchW); x += 8) {
                const uint8_t* p = d + y * stride + x * 3;
                sum += p[0] + 2u * p[1] + p[2];
                cnt += 4;
            }
        bandLuma = cnt ? static_cast<float>(sum) / cnt : NAN;
    });
    det->setResultSink([&](const LaneResult& r) {
        const int64_t now = monoNs();
        int dumpId = 0;
        if (dumpsOn.load(std::memory_order_relaxed) && now >= nextPeriodic) {
            bool take = true;
            // CSI: the ring holds 30 of 60 fps — prefer a lane frame whose
            // PTS is (or will be) in it, unless the dump is 5 s overdue (then
            // the nearest frame).  Not in the ring yet: the encode branch
            // keeps one frame per ring interval after its newest, so a PTS
            // about k intervals on will arrive; half an interval on will not.
            if (o.cam == "csi" && r.timing.ptsNs >= 0 && now - nextPeriodic < 5 * kSec) {
                JFrame f;
                int64_t d = 0;
                if (!ring.find(r.timing.ptsNs, f, d, false)) {
                    const int64_t iv = static_cast<int64_t>(1e9 / std::max(1.0, rig->ringFps()));
                    if (!ring.find(r.timing.ptsNs, f, d, true) || d > 0) {
                        take = false;                    // passed (or no ring yet)
                    } else {
                        const int64_t delta = -d;        // this PTS − the ring's newest
                        const int64_t k = (delta + iv / 2) / iv;
                        take = k >= 1 && std::llabs(delta - k * iv) < iv / 4;
                    }
                }
            }
            if (take) {
                dumpId = ++nextDumpId;
                pendingDumpSeq = r.sequence;
                pendingDumpId = dumpId;
                nextPeriodic = now + dumpPeriod;
            }
        }
        frames.row(frameRow(o.cam, r, bandLuma, dumpId, live));
        ResultSig s;
        s.mono = r.timing.doneMonoNs >= 0 ? r.timing.doneMonoNs : now;
        if (r.timing.captureMonoNs >= 0 && r.timing.doneMonoNs >= 0) s.e2eMs = (r.timing.doneMonoNs - r.timing.captureMonoNs) / 1e6f;
        s.numLanes = r.numLanes;
        s.ego = r.currentLaneIndex;
        s.off = r.lateralOffset;
        s.offValid = r.lateralValid;
        s.slotOk = slotOrderOk(r);
        s.nonFinite = r.nonFinite;
        s.seq = r.sequence;
        sigs.push(s);
    });
    LaneDetectorTestHook::onInference(*det, [&](const LaneDetectorTestHook::InferenceView& v) {
        const LaneResult& r = *v.result;
        tensors.put(r.sequence, r.timing.ptsNs, r.timing.captureMonoNs, v.outputs, v.outputFloats);
        if (pendingDumpSeq != r.sequence) return;
        pendingDumpSeq = 0;
        DumpJob j;
        j.id = pendingDumpId;
        j.trigger = "periodic";
        j.t.seq = r.sequence;
        j.t.pts = r.timing.ptsNs;
        j.t.captureMono = r.timing.captureMonoNs;
        size_t total = 0;
        for (int i = 0; i < 4; ++i) { j.t.counts[i] = static_cast<uint32_t>(v.outputFloats[i]); total += v.outputFloats[i]; }
        j.t.data.resize(total);
        size_t off = 0;
        for (int i = 0; i < 4; ++i) {
            std::memcpy(j.t.data.data() + off, v.outputs[i], v.outputFloats[i] * sizeof(float));
            off += v.outputFloats[i];
        }
        j.band.assign(v.frame, v.frame + v.frameBytes);
        j.bandW = static_cast<int>(geo.branchW);
        j.bandH = static_cast<int>(geo.branchH);
        dumper.enqueue(std::move(j));
    });

    {   // run.json (run_i<K>.json for a restarted instance)
        std::ofstream rj(dir + (o.instance > 0 ? "/run_i" + std::to_string(o.instance) + ".json" : "/run.json"));
        rj << "{\n \"cam\": " << jsonStr(o.cam) << ",\n \"instance\": " << o.instance << ",\n \"engine\": " << jsonStr(o.engine)
           << ",\n \"engine_bytes\": " << fs::file_size(o.engine, ec)
           << ",\n \"src\": [" << srcW << ", " << srcH << "],\n \"crop\": [" << o.cropTop << ", " << o.cropBottom
           << "],\n \"ref_y\": " << o.refY << ",\n \"target_hz\": " << o.hz << ",\n \"branch_fps\": " << o.branchFps
           << ",\n \"rate_gate\": " << jsonStr(o.rateGate) << ",\n \"review_fps\": " << o.reviewFps
           << ",\n \"jpeg_q\": " << o.jpegQ << ",\n \"usb_decode\": " << jsonStr(o.usbDecode)
           << ",\n \"usb_exposure\": " << jsonStr(o.usbExposure) << ",\n \"target_luma\": " << o.targetLuma
           << ",\n \"focus\": " << o.focus << ",\n \"lanes_off_s\": " << o.lanesOffS
           << ",\n \"lanes_schedule\": " << jsonStr(o.lanesSchedule)
           << ",\n \"capture_branch_disabled\": " << (o.cam == "csi" ? "true" : "null")
           << ",\n \"ring_fps\": " << ro.ringFps << ",\n \"clip_ring_mb\": " << o.clipRingMb
           << ",\n \"clip_budget_gb_h\": " << o.clipBudgetGbH << ",\n \"clips\": " << (o.clips ? "true" : "false")
           << ",\n \"random_clip_min\": " << o.randomClipMin << ",\n \"tensor_hz\": " << o.tensorHz
           << ",\n \"event_dumps\": " << (o.eventDumps ? "true" : "false") << ",\n \"stats_hz\": " << o.statsHz
           << ",\n \"mount\": " << (o.mount ? "true" : "false") << ",\n \"mount_roi\": [" << o.mountRoi.y0 << ", "
           << o.mountRoi.y1 << ", " << o.mountRoi.x0 << ", " << o.mountRoi.x1 << "],\n \"mount_ref\": " << jsonStr(o.mountRef)
           << ",\n \"geometry\": {\"crop_top_px\": " << geo.cropTopPx << ", \"crop_bottom_px\": " << geo.cropBottomPx
           << ", \"branch\": [" << geo.branchW << ", " << geo.branchH << "], \"eff_ratio\": " << fmtF(geo.effRatio)
           << ", \"pre_y_scale\": " << fmtF(geo.preYScale) << ", \"pre_y_off\": " << fmtF(geo.preYOff)
           << ", \"implied_h\": " << fmtF(geo.impliedH) << ", \"implied_top_src\": " << fmtF(geo.impliedTopSrc)
           << ", \"ref_row_src\": " << fmtF(geo.refRowSrc) << "},\n \"start_mono_ns\": " << startMono
           << ",\n \"start_realtime_ns\": " << startReal << "\n}\n";
    }

    // UGREEN: fixed focus + frame-rate-priority exposure, handed back at exit.
    UvcFocusControl focus;
    UvcExposureControl exposure;
    int ctrlFd = -1;
    if (o.cam == "usb") {
        if (o.focus >= 0) {
            if (!focus.open(o.usbDev, o.focus, logCb, why)) ev.add("ctrl", "focus_not_held", why);
            else ev.add("ctrl", "focus_held", std::to_string(focus.position()));
        }
        ctrlFd = ::open(o.usbDev.c_str(), O_RDWR | O_NONBLOCK);
    }

    // ── clips ────────────────────────────────────────────────────────────────
    ClipBudget budget(o.clipBudgetGbH, 1200, 1.0);   // 20 min of budget, full at the start
    ClipPlanner planner;
    ClipWriter clipper(clipsDir, o.cam, clipsIdx, rig->ringW(), rig->ringH(), rig->ringFps());
    bool clipsOn = o.clips;
    uint64_t clipsKept = 0;
    auto ringRate = [&]() {                      // bytes/s the ring receives (clip size estimate)
        const double span = ring.spanS();
        return span > 1 ? ring.bytes() / span : 3e6;
    };
    std::map<std::string, int64_t> lastTrig;     // rate limit per internal trigger type
    auto clipTrigger = [&](const std::string& name, int prio, int64_t mono, int64_t minGapNs = 30 * kSec) {
        if (!clipsOn) return;
        auto it = lastTrig.find(name);
        if (it != lastTrig.end() && mono - it->second < minGapNs) return;
        lastTrig[name] = mono;
        const int id = planner.trigger(name, prio, mono, ringRate(), budget, clipsOn);
        ev.add(id > 0 ? "clip_trigger" : "clip_refused", name, "clip " + std::to_string(id) + " prio " + std::to_string(prio));
    };
    std::map<std::string, int64_t> lastEventDump;
    auto eventDump = [&](const std::string& type) {
        if (!o.eventDumps || !dumpsOn.load()) return;
        const int64_t now = monoNs();
        auto it = lastEventDump.find(type);
        if (it != lastEventDump.end() && now - it->second < 30 * kSec) return;
        const auto sets = tensors.snapshot();
        if (sets.empty()) return;
        double est = 0;
        for (const auto& t : sets) est += t.data.size() * 4.0 + ringRate() / std::max(1.0, rig->ringFps());
        if (!budget.admit(kFault, est, now / 1e9)) { ev.add("dump_refused", type); return; }
        lastEventDump[type] = now;
        int first = 0;
        for (const auto& t : sets) {
            DumpJob j;
            j.id = ++nextDumpId;
            if (!first) first = j.id;
            j.trigger = "event:" + type;
            j.t = t;
            dumper.enqueue(std::move(j));
        }
        ev.add("event_dump", type, "dumps " + std::to_string(first) + "+" + std::to_string(sets.size()));
    };

    // ── external triggers (RUN_DIR/triggers.csv: mono_ns,source,type,priority,detail) ──
    const std::string trigPath = o.out + "/triggers.csv";
    std::ifstream trigF;
    std::streamoff trigOff = 0;
    bool trigSkipOld = fs::exists(trigPath);     // only lines written after we started
    struct LcWatch { int64_t deadline; int egoAtOn; float offAtOn; bool crossed; std::string side; };
    std::vector<LcWatch> lcWatch;
    auto readTriggers = [&]() {
        if (!trigF.is_open()) {
            if (!fs::exists(trigPath)) return;
            trigF.open(trigPath);
            if (!trigF) return;
            if (trigSkipOld) { trigF.seekg(0, std::ios::end); trigOff = trigF.tellg(); }
        }
        trigF.clear();
        trigF.seekg(trigOff);
        std::string line;
        while (std::getline(trigF, line)) {
            if (trigF.eof()) break;                      // partial line: read again next time
            trigOff = trigF.tellg();
            std::stringstream ss(line);
            std::string monoS, src, type, prioS, detail;
            std::getline(ss, monoS, ','); std::getline(ss, src, ','); std::getline(ss, type, ',');
            std::getline(ss, prioS, ','); std::getline(ss, detail);
            if (monoS.empty() || !std::isdigit(static_cast<unsigned char>(monoS[0]))) continue;
            const int64_t mono = std::stoll(monoS);
            const int prio = prioS.empty() ? kRest : std::stoi(prioS);
            ev.add("trigger", type, src + " " + detail);
            if (type == "stalk_on") lcWatch.push_back({ mono + 5 * kSec, -99, NAN, false, detail });
            if (clipsOn) {
                const int id = planner.trigger(type, prio, mono, ringRate(), budget, clipsOn);
                if (id <= 0) ev.add("clip_refused", type, src);
            }
        }
    };

    // ── run ──────────────────────────────────────────────────────────────────
    // A restarted instance (--instance K > 0) resumes lanes at once: the
    // lead-in baseline belongs to the run's start, not to a mid-drive restart.
    const int lanesOffS = o.instance > 0 ? 0 : o.lanesOffS;
    auto wantLanes = [&](int64_t now) {
        const double el = (now - startMono) / 1e9;
        return sched.empty() ? el >= lanesOffS : scheduledLanes(sched, el);
    };
    bool lanesOn = wantLanes(monoNs());
    int part = o.instance * 100, restarts = 0, failures = 0;
    bool running = false, lost = false;
    AssWriter ass;
    auto startCamera = [&]() -> bool {
        stageName = "start";
        deadlineNs = monoNs() + 90'000'000'000LL;
        GstElement* bin = det->createBin();
        if (!bin) { ev.add("cam_error", "createBin"); return false; }
        std::string w;
        const bool ok = rig->start(bin, lanesOn, part, w);
        if (!ok) ev.add("cam_error", "start", w);
        else det->start();
        ass.close();
        ass.open(reviewDir + "/" + o.cam + "_p" + std::to_string(part) + ".ass", rig->ringW(), rig->ringH());
        deadlineNs = INT64_MAX;
        return ok;
    };
    running = startCamera();
    if (running && o.cam == "usb" && o.usbExposure == "framerate") {
        if (!exposure.open(o.usbDev, static_cast<float>(o.usbFps), o.targetLuma, logCb, why))
            ev.add("ctrl", "exposure_camera_kept", why);
        else {
            exposure.enableNightMode(NightModeSwitch::Policy{});
            ev.add("ctrl", "exposure_framerate", std::to_string(o.targetLuma));
        }
    }
    if (running) ev.add("healthy", lanesOn ? "lanes_on" : "lanes_off");

    // Mount reference: given (calibration / pre-flight) or the parked lead-in.
    // The verdict is authoritative (FAIL alert, clip, event dump) only when
    // the region is known to be car-fixed: a calibration's roi_<cam>.txt
    // (roi_calib.py writes one only when it found the hood edge) or an
    // explicit --mount-roi.  Otherwise the monitor reports: a region of road
    // (the UGREEN's 10-03 view has no hood or dashboard in it) decorrelates as
    // soon as the car moves and would read as a lost view.
    MountMonitor mount;
    std::vector<Gray> refFrames;
    bool mountAuth = o.mountRoiSet;
    MountRoi mountRoi = o.mountRoi;
    // A restarted instance reuses the reference the run started with (a new
    // "parked" reference taken mid-drive would hide a shift that already
    // happened); mount_ref/auth marks whether it was car-fixed.
    std::string mountRefDir = o.mountRef;
    if (mountRefDir.empty() && o.instance > 0 && fs::exists(mountDir + "/ref.pgm")) {
        mountRefDir = mountDir;
        if (!o.mountRoiSet && fs::exists(mountDir + "/auth")) {
            const std::string r = readSmall(mountDir + "/roi.txt");
            if (!r.empty() && parseRoi(r, mountRoi)) mountAuth = true;
        }
    }
    if (o.mount && !mountRefDir.empty()) {
        Gray g;
        const std::string rd = mountRefDir;
        const std::string roiTxt = readSmall(rd + "/roi_" + o.cam + ".txt");
        if (!roiTxt.empty() && !o.mountRoiSet && parseRoi(roiTxt, mountRoi)) mountAuth = true;
        if ((loadPgm(rd + "/ref_" + o.cam + ".pgm", g) || loadPgm(rd + "/ref.pgm", g)) &&
            mount.setReference(g, mountRoi, 1.0f / rig->statsScale())) {
            if (rd != mountDir) {
                savePgm(mountDir + "/ref.pgm", g);
                writeAtomic(mountDir + "/roi.txt", fmtF(mountRoi.y0) + "," + fmtF(mountRoi.y1) + "," +
                            fmtF(mountRoi.x0) + "," + fmtF(mountRoi.x1) + "\n");
                if (mountAuth) writeAtomic(mountDir + "/auth", "car-fixed\n");
            }
            ev.add("mount_ref", "loaded", rd + " std " + fmtF(mount.referenceStd()) +
                   (mountAuth ? " (car-fixed roi)" : " (roi unverified: report only)"));
        } else {
            ev.add("mount_ref", "load_failed", rd);
        }
    }

    const int64_t endNs = o.minutes > 0 ? monoNs() + static_cast<int64_t>(o.minutes * 60e9) : INT64_MAX;
    int64_t lastTick = monoNs(), lastWallMinusMono = realNs() - monoNs(), last5Hz = 0, lastMount = 0;
    uint64_t lastProcessed = 0, lastOffered = 0, lastPulled = 0;
    int64_t lastPullProgressNs = monoNs(), nextRetryNs = 0;
    std::deque<std::pair<double, uint64_t>> fpsWin;
    float lastLuma = NAN;
    Gray prevStats;
    const int backoff[] = { 2, 4, 8, 16, 30 };
    // In-process restarts: 5 for the UGREEN (unplug / replug recovers in
    // place); 2 for the CSI — after an Argus sensor error ("waitCsiFrameStart
    // timeout ... Sensor GUID 0 is in error state") only a fresh process gets
    // a working session (measured 2026-10-26: 5 of 5 in-process restarts
    // failed, a new process at once worked).
    const int maxTries = o.cam == "csi" ? 2 : 5;
    std::mt19937 rng(static_cast<unsigned>(startReal));
    int64_t nextRandomClip = o.randomClipMin > 0
        ? startMono + static_cast<int64_t>(std::uniform_real_distribution<double>(0.3, 1.0)(rng) * o.randomClipMin * 60e9)
        : INT64_MAX;
    // Result-derived signals.
    std::deque<std::pair<int64_t, float>> e2eWin;     // (mono, e2e) for the 60 s p50
    float e2eP50 = NAN;
    int spikeRun = 0;
    std::deque<int64_t> laneChanges;
    int lastNumLanes = -1;
    ResultSig lastSig;
    bool haveLastSig = false;
    uint64_t teleports = 0, slotViolations = 0, flickerBursts = 0;
    std::deque<ResultSig> recent;                     // 10 s, for the missed-LC check
    LaneStats lastStats = det->stats();
    int degrade = 0, laneHz = static_cast<int>(o.branchFps);
    int stillsTaken = 0;
    int64_t nextStill = startMono + 3 * kSec;
    std::string mountState = o.mount ? "no_ref" : "off";
    MountMonitor::Verdict lastVerdict;
    MountSample lastMountSample;
    CsvWriter stillsIdx;
    if (o.stills > 0) stillsIdx.open(stillsDir + "/stills.csv", "still,pts_ns,capture_mono_ns,file");
    std::deque<float> latWin10;                       // 10 s of e2e for status.json
    double bannerHz = 0;
    int64_t lastAssPts = -1;

    while (!g_signal && monoNs() < endNs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const int64_t now = monoNs();

        // ── stats tap: exposure loop, image_stats.csv, mount monitor ──────────
        Gray g;
        int64_t gPts = -1;
        if (running && rig->statsFrame(g, gPts) && !g.px.empty()) {
            uint64_t sum = 0;
            for (uint8_t p : g.px) sum += p;
            const float luma = static_cast<float>(sum) / g.px.size();
            lastLuma = luma;
            const double t = now / 1e9;
            fpsWin.emplace_back(t, rig->offered());
            while (fpsWin.size() > 2 && t - fpsWin.front().first > 2.5) fpsWin.pop_front();
            const double span = t - fpsWin.front().first;
            const float fps = span >= 1.0 ? static_cast<float>((fpsWin.back().second - fpsWin.front().second) / span) : NAN;
            if (exposure.isOpen()) exposure.onLuma(luma, fps, t);

            // image_stats: band rows = the lane crop in tap coordinates.
            const int b0 = std::clamp(static_cast<int>(o.cropTop * g.h), 0, g.h - 1);
            const int b1 = std::clamp(static_cast<int>((1.0f - o.cropBottom) * g.h), b0 + 1, g.h);
            std::vector<uint8_t> band;
            band.reserve(static_cast<size_t>(b1 - b0) * g.w);
            for (int y = b0; y < b1; ++y) band.insert(band.end(), g.px.begin() + y * g.w, g.px.begin() + (y + 1) * g.w);
            double bandMean = 0;
            for (uint8_t p : band) bandMean += p;
            bandMean /= std::max<size_t>(1, band.size());
            std::nth_element(band.begin(), band.begin() + band.size() / 10, band.end());
            const int p10 = band[band.size() / 10];
            std::nth_element(band.begin(), band.begin() + band.size() * 9 / 10, band.end());
            const int p90 = band[band.size() * 9 / 10];
            double sky = 0;
            const int skyRows = std::max(1, g.h * 15 / 100);
            for (int i = 0; i < skyRows * g.w; ++i) sky += g.px[i];
            sky /= skyRows * g.w;
            // Noise: MAD of horizontal neighbour differences (σ of i.i.d. noise).
            std::vector<uint8_t> diffs;
            for (int y = 0; y < g.h; y += 4)
                for (int x = 0; x + 1 < g.w; x += 2)
                    diffs.push_back(static_cast<uint8_t>(std::abs(g.at(x + 1, y) - g.at(x, y))));
            std::nth_element(diffs.begin(), diffs.begin() + diffs.size() / 2, diffs.end());
            const double noise = diffs.empty() ? NAN : diffs[diffs.size() / 2] * 1.4826 / std::sqrt(2.0);
            double motion = NAN;
            if (prevStats.w == g.w && prevStats.h == g.h) {
                double m = 0;
                for (size_t i = 0; i < g.px.size(); i += 3) m += std::abs(g.px[i] - prevStats.px[i]);
                motion = m / ((g.px.size() + 2) / 3);
            }
            prevStats = g;
            imgStats.row(std::to_string(now) + "," + (gPts >= 0 ? std::to_string(gPts) : "") + "," + fmtF(bandMean) +
                         "," + std::to_string(p10) + "," + std::to_string(p90) + "," + fmtF(sky) + "," + fmtF(luma) + "," +
                         fmtF(noise) + "," + fmtF(motion));

            if (o.mount && now - lastMount >= 900'000'000LL) {
                lastMount = now;
                if (!mount.hasReference()) {
                    refFrames.push_back(g);
                    if (static_cast<int>(refFrames.size()) >= std::max(1, o.mountRefS)) {
                        const Gray ref = medianOf(refFrames);
                        refFrames.clear();
                        if (mount.setReference(ref, mountRoi, 1.0f / rig->statsScale())) {
                            savePgm(mountDir + "/ref.pgm", ref);
                            writeAtomic(mountDir + "/roi.txt", fmtF(mountRoi.y0) + "," + fmtF(mountRoi.y1) + "," +
                                        fmtF(mountRoi.x0) + "," + fmtF(mountRoi.x1) + "\n");
                            if (mountAuth) writeAtomic(mountDir + "/auth", "car-fixed\n");
                            ev.add("mount_ref", "auto", "median of " + std::to_string(o.mountRefS) +
                                   " parked frames; template std " + fmtF(mount.referenceStd()) +
                                   (mountAuth ? " (car-fixed roi)" : " (roi unverified: report only)"));
                            mountState = "ok";
                        } else {
                            ev.add("mount_ref", "rejected", "roi too small");
                            mountState = "off";
                        }
                    }
                } else {
                    const MountSample s = mount.measure(g);
                    const auto v = mount.feed(now, s);
                    lastMountSample = s;
                    lastVerdict = v;
                    mountState = v.shifted ? std::string(mountAuth ? "FAIL" : "shift_unverified") + (mount.active() ? "" : "_back")
                                           : s.lost ? "lost" : !s.ok ? "no_measure" : "ok";
                    mountCsv.row(std::to_string(now) + "," + (s.ok ? fmtF(s.dx) : "") + "," + (s.ok ? fmtF(s.dy) : "") +
                                 "," + (s.ok ? fmtF(s.rollDeg) : "") + ",edge," + fmtF(s.conf) + "," +
                                 (v.have ? fmtF(v.med5Dx) + "," + fmtF(v.med5Dy) + "," + fmtF(v.med5Roll) : ",,") + ",," +
                                 std::to_string(v.shifted) + "," + std::to_string(v.exceeding) + "," +
                                 std::to_string(s.lost) + "," + fmtF(s.featureRatio));
                    if (v.onEdge) {
                        ev.add("mount_shift_on", mountAuth ? "" : "unverified_roi", "med5 dx " + fmtF(v.med5Dx) + " dy " +
                               fmtF(v.med5Dy) + " roll " + fmtF(v.med5Roll) + (s.lost ? " (view lost)" : ""));
                        if (mountAuth) {
                            clipTrigger("mount_shift", kFault, now, 0);
                            eventDump("mount_shift");
                        }
                    }
                    if (v.offEdge) ev.add("mount_shift_off", "", "med5 dy " + fmtF(v.med5Dy));
                }
            }
        }

        // ── lanes schedule / lead-in ──────────────────────────────────────────
        const bool want = wantLanes(now) && !(o.cam == "usb" && degrade >= 4);
        if (want != lanesOn) {
            lanesOn = want;
            if (running) rig->setLanes(lanesOn);
            lastPullProgressNs = now;          // the stall clock starts with the lanes
            ev.add(lanesOn ? "lanes_on" : "lanes_off");
        }

        // ── health: camera error, or no frame reaching the detector for stallS ─
        const uint64_t pulled = det->stats().pulled;
        if (pulled != lastPulled) { lastPulled = pulled; lastPullProgressNs = now; }
        if (running) {
            std::string w;
            const bool stalled = lanesOn && now - lastPullProgressNs > static_cast<int64_t>(o.stallS) * kSec;
            if (rig->failed(w) || stalled) {
                ev.add(stalled ? "stall" : "cam_error", "", stalled ? "no frame reached the detector" : w);
                clipTrigger(stalled ? "stall" : "cam_error", kFault, now, 0);
                stageName = "restart stop";
                deadlineNs = now + 30'000'000'000LL;
                rig->stop();
                det->stop();
                deadlineNs = INT64_MAX;
                running = false;
                nextRetryNs = now + backoff[std::min(failures, 4)] * kSec;
            }
        } else if (!lost && now >= nextRetryNs) {
            if (failures >= maxTries) {
                lost = true;
                ev.add("cam_lost", std::to_string(failures), "finalising; exit 3 (the supervisor restarts the container)");
                break;
            } else {
                ++failures;
                ++restarts;
                ++part;
                running = startCamera();
                ev.add("cam_restart", std::to_string(restarts), running ? "ok" : "failed");
                if (running) { failures = 0; lastPullProgressNs = monoNs(); if (laneHz != static_cast<int>(o.branchFps)) rig->setLaneHz(laneHz); }
                else nextRetryNs = monoNs() + backoff[std::min(failures, 4)] * kSec;
            }
        }

        // ── result signals: latency spikes, flicker, teleports, slot order ───
        for (const ResultSig& s : sigs.drain()) {
            if (std::isfinite(s.e2eMs)) {
                e2eWin.emplace_back(s.mono, s.e2eMs);
                latWin10.push_back(s.e2eMs);
                if (latWin10.size() > 200) latWin10.pop_front();
                if (std::isfinite(e2eP50) && s.e2eMs > 3 * e2eP50) {
                    if (++spikeRun == 5) {
                        ev.add("latency_spike", fmtF(s.e2eMs), "p50 " + fmtF(e2eP50));
                        clipTrigger("latency_spike", kFault, s.mono);
                    }
                } else {
                    spikeRun = 0;
                }
            }
            if (lastNumLanes >= 0 && s.numLanes != lastNumLanes) laneChanges.push_back(s.mono);
            lastNumLanes = s.numLanes;
            while (!laneChanges.empty() && s.mono - laneChanges.front() > 2 * kSec) laneChanges.pop_front();
            if (laneChanges.size() >= 4) {
                ++flickerBursts;
                laneChanges.clear();
                ev.add("flicker_burst");
                clipTrigger("flicker", kRest, s.mono, 60 * kSec);
                eventDump("flicker");
            }
            if (haveLastSig && s.offValid && lastSig.offValid && s.ego == lastSig.ego &&
                s.mono - lastSig.mono <= 120'000'000LL && std::fabs(s.off - lastSig.off) > 0.5f) {
                ++teleports;
                eventDump("teleport");
            }
            if (!s.slotOk) { ++slotViolations; eventDump("slot_order"); }
            if (s.nonFinite > 0) eventDump("nonfinite");
            for (auto& w : lcWatch) {
                if (w.egoAtOn == -99) { w.egoAtOn = s.ego; w.offAtOn = s.offValid ? s.off : NAN; }
                else if (s.ego != w.egoAtOn || (s.offValid && std::isfinite(w.offAtOn) && std::fabs(s.off - w.offAtOn) > 1.0f))
                    w.crossed = true;
            }
            lastSig = s;
            haveLastSig = true;
            recent.push_back(s);
            while (recent.size() > 1 && s.mono - recent.front().mono > 10 * kSec) recent.pop_front();
        }
        for (auto it = lcWatch.begin(); it != lcWatch.end();) {
            if (now >= it->deadline) {
                if (!it->crossed) { ev.add("missed_lc", it->side); eventDump("missed_lc"); }
                it = lcWatch.erase(it);
            } else {
                ++it;
            }
        }
        if (now >= nextRandomClip) {
            planner.trigger("random", kRest, now, ringRate(), budget, clipsOn);
            nextRandomClip = now + static_cast<int64_t>(o.randomClipMin * 60e9);
        }
        readTriggers();

        // ── clips: fill open windows, hand finished ones to the writer ───────
        if (clipsOn) {
            planner.collect(ring);
            for (ClipJob& j : planner.finished(ring.newestMono(), now)) {
                budget.adjust(static_cast<double>(j.bytes) - j.estBytes);
                const std::string info = std::to_string(j.frames.size()) + " frames " + std::to_string(j.bytes >> 20) +
                                         " MiB " + j.triggers;
                const int id = j.id;
                if (clipper.enqueue(std::move(j))) { ++clipsKept; ev.add("clip_kept", std::to_string(id), info); }
                else ev.add("clip_dropped", std::to_string(id), "writer queue full");
            }
        }

        // ── stills (calibration) ──────────────────────────────────────────────
        if (stillsTaken < o.stills && running && now >= nextStill) {
            JFrame f;
            int64_t d = 0;
            if (ring.find(INT64_MAX / 2, f, d, true)) {
                char nm[64];
                std::snprintf(nm, sizeof(nm), "still_%02d.jpg", stillsTaken);
                if (std::FILE* sf = std::fopen((stillsDir + "/" + nm).c_str(), "wb")) {
                    std::fwrite(f.data->data(), 1, f.data->size(), sf);
                    std::fclose(sf);
                    stillsIdx.row(std::to_string(stillsTaken) + "," + std::to_string(f.pts) + "," + std::to_string(f.mono) + "," + nm);
                    ++stillsTaken;
                }
            }
            nextStill = now + kSec;
        }

        // ── 5 Hz: live CAN state, .ass banner ─────────────────────────────────
        if (now - last5Hz >= 200'000'000LL) {
            last5Hz = now;
            const std::string js = readSmall(o.out + "/can/live_state.json");
            double sp = NAN, mono = 0, tl = 0, tr = 0;
            std::string gear;
            if (!js.empty() && jsonNumber(js, "mono_ns", mono)) {
                live.have = true;
                live.speed = jsonNumber(js, "speed_kmh", sp) ? static_cast<float>(sp) : NAN;
                live.gear = jsonString(js, "gear", gear) && gear.size() == 1 ? gear[0] : 0;
                jsonNumber(js, "turn_l", tl);
                jsonNumber(js, "turn_r", tr);
                live.stalk = tl > 0 ? -1 : tr > 0 ? 1 : 0;
                live.ageMs = (now - static_cast<int64_t>(mono)) / 1'000'000;
            }
            const LaneResult r = det->poll();
            if (running && r.valid && r.timing.ptsNs >= 0 && r.timing.ptsNs != lastAssPts) {
                lastAssPts = r.timing.ptsNs;
                char off[16] = "--", lat[16] = "--", spd[16] = "--";
                if (r.lateralValid) std::snprintf(off, sizeof(off), "%+.2f", r.lateralOffset);
                if (r.timing.captureMonoNs >= 0 && r.timing.doneMonoNs >= 0)
                    std::snprintf(lat, sizeof(lat), "%.0f", (r.timing.doneMonoNs - r.timing.captureMonoNs) / 1e6);
                if (live.have.load() && std::isfinite(live.speed.load()))
                    std::snprintf(spd, sizeof(spd), "%.0f", live.speed.load());
                const int gch = live.gear.load(), stk = live.stalk.load();
                char b[200];
                std::snprintf(b, sizeof(b), "%s L%d E%s off%s %.1fHz %sms | CAN* %s %c %s",
                              o.cam == "csi" ? "CSI" : "USB", r.numLanes,
                              r.currentLaneIndex >= 0 ? std::to_string(r.currentLaneIndex).c_str() : "-", off,
                              bannerHz, lat, spd, gch > 0 ? static_cast<char>(gch) : '-',
                              stk < 0 ? "<" : stk > 0 ? ">" : "-");
                ass.line(r.timing.ptsNs, 200'000'000LL, b);
            }
        }

        if (now - lastTick < kSec) continue;
        // ── 1 Hz rows ────────────────────────────────────────────────────────
        const double dt = (now - lastTick) / 1e9;
        lastTick = now;
        const LaneStats s = det->stats();
        const uint64_t offered = rig->offered();
        const std::string state = lost ? "LOST" : running ? "RUNNING" : "RESTARTING";
        const double effHz = (s.processed - lastProcessed) / dt;
        const double srcFps = (offered - lastOffered) / dt;
        bannerHz = effHz;

        // 60 s latency p50 (spike reference).
        while (!e2eWin.empty() && now - e2eWin.front().first > 60 * kSec) e2eWin.pop_front();
        if (e2eWin.size() >= 20) {
            std::vector<float> v;
            v.reserve(e2eWin.size());
            for (const auto& p : e2eWin) v.push_back(p.second);
            std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
            e2eP50 = v[v.size() / 2];
        }
        // An error counter that moved = a fault (event + clip + event dump).
        const uint64_t errNow = s.mapFail + s.sizeMismatch + s.h2dFail + s.preFail + s.trtFail + s.d2hFail + s.syncFail + s.nonFinite;
        const uint64_t errLast = lastStats.mapFail + lastStats.sizeMismatch + lastStats.h2dFail + lastStats.preFail +
                                 lastStats.trtFail + lastStats.d2hFail + lastStats.syncFail + lastStats.nonFinite;
        if (errNow > errLast) {
            ev.add("counter_inc", std::to_string(errNow - errLast));
            clipTrigger("counter_inc", kFault, now);
            eventDump("counter_inc");
        }
        lastStats = s;

        // Degrade ladder and lane-rate control from the supervisor.
        const std::string dg = readSmall(o.out + "/ctl/degrade");
        const int want_dg = dg.empty() ? 0 : std::atoi(dg.c_str());
        if (want_dg > degrade) {
            for (int l = degrade + 1; l <= want_dg; ++l) ev.add("degrade_" + std::to_string(l));
            degrade = want_dg;
            if (degrade >= 1 && clipsOn) { clipsOn = false; planner.dropAll(); }
            if (degrade >= 2) { dumpsOn = false; if (o.stills <= stillsTaken) ring.setEnabled(false); }
            if (degrade >= 3 && o.cam == "csi") rig->setReview(false);
            if (degrade >= 5 && o.cam == "usb") rig->setReview(false);
        }
        const std::string lh = readSmall(o.out + "/ctl/" + o.cam + "_lane_hz");
        const int wantHz = lh.empty() ? static_cast<int>(o.branchFps) : std::atoi(lh.c_str());
        if (o.cam == "usb" && wantHz > 0 && wantHz != laneHz) {
            laneHz = wantHz;
            rig->setLaneHz(laneHz);
            ev.add("degrade_gpu", std::to_string(laneHz), "UGREEN lane rate");
        }

        counters.row(std::to_string(now) + "," + state + "," + std::to_string(lanesOn) + "," +
                     std::to_string(offered) + "," + std::to_string(s.pulled) + "," + std::to_string(s.gated) + "," +
                     std::to_string(s.processed) + "," + std::to_string(s.mapFail) + "," +
                     std::to_string(s.sizeMismatch) + "," + std::to_string(s.h2dFail) + "," +
                     std::to_string(s.preFail) + "," + std::to_string(s.trtFail) + "," +
                     std::to_string(s.d2hFail) + "," + std::to_string(s.syncFail) + "," +
                     std::to_string(s.nonFinite) + "," + fmtF(effHz) + "," + fmtF(srcFps) + "," +
                     std::to_string(restarts) + "," + std::to_string(frames.drops()) + "," +
                     std::to_string(procStatusKb("VmRSS:")) + "," + std::to_string(countDir("/proc/self/task")) + "," +
                     std::to_string(countDir("/proc/self/fd")) + "," + std::to_string(rig->ringFrames()) + "," +
                     std::to_string(ring.bytes()) + "," + fmtF(ring.spanS()) + "," + std::to_string(clipsKept) + "," +
                     std::to_string(planner.refused()) + "," + std::to_string(planner.overflowFrames()) + "," +
                     std::to_string(clipper.dropped()) + "," + std::to_string(dumper.written()) + "," +
                     std::to_string(dumper.dropped()) + "," + std::to_string(sigs.dropped()) + "," +
                     std::to_string(degrade) + "," + std::to_string(laneHz));
        lastProcessed = s.processed;
        lastOffered = offered;

        if (o.cam == "usb") {
            ctrl.row(std::to_string(now) + "," + ctrlStr(readCtrl(ctrlFd, V4L2_CID_EXPOSURE_ABSOLUTE)) + ",100us," +
                     ctrlStr(readCtrl(ctrlFd, V4L2_CID_GAIN)) + ",raw," +
                     ctrlStr(readCtrl(ctrlFd, V4L2_CID_EXPOSURE_AUTO)) + "," +
                     ctrlStr(readCtrl(ctrlFd, V4L2_CID_FOCUS_ABSOLUTE)) + "," +
                     ctrlStr(readCtrl(ctrlFd, V4L2_CID_FOCUS_AUTO)) + "," + fmtF(srcFps) + "," + fmtF(lastLuma) +
                     "," + (exposure.isOpen() ? std::to_string(exposure.nightMode()) : "") + ",v4l2");
        } else {
            ctrl.row(std::to_string(now) + ",,lines,,0.1dB,,,," + fmtF(srcFps) + "," + fmtF(lastLuma) + ",,unavailable");
        }

        const int64_t mono = monoNs(), real = realNs(), wmm = real - mono;   // sampled together
        const double stepMs = (wmm - lastWallMinusMono) / 1e6;
        clockCsv.row(std::to_string(mono) + "," + std::to_string(real) + "," + std::to_string(bootNs()) + "," +
                     std::to_string(wmm) + "," + fmtF(stepMs));
        if (std::fabs(stepMs) > 50) ev.add("clock_step", fmtF(stepMs));
        lastWallMinusMono = wmm;

        // status.json for the supervisor.
        std::vector<float> lw(latWin10.begin(), latWin10.end());
        float p50 = NAN, p95 = NAN;
        if (!lw.empty()) {
            std::sort(lw.begin(), lw.end());
            p50 = lw[lw.size() / 2];
            p95 = lw[std::min(lw.size() - 1, lw.size() * 95 / 100)];
        }
        const LaneResult lr = det->poll();
        std::ostringstream st;
        st << "{\"mono_ns\": " << now << ", \"cam\": " << jsonStr(o.cam) << ", \"state\": " << jsonStr(state)
           << ", \"lanes_on\": " << lanesOn << ", \"eff_hz\": " << (fmtF(effHz).empty() ? "null" : fmtF(effHz))
           << ", \"src_fps\": " << (fmtF(srcFps).empty() ? "null" : fmtF(srcFps))
           << ", \"target_hz\": " << (o.cam == "usb" ? laneHz : static_cast<int>(o.hz))
           << ", \"lat_p50_ms\": " << (std::isfinite(p50) ? fmtF(p50) : "null")
           << ", \"lat_p95_ms\": " << (std::isfinite(p95) ? fmtF(p95) : "null")
           << ", \"num_lanes\": " << static_cast<int>(lr.numLanes) << ", \"ego\": " << static_cast<int>(lr.currentLaneIndex)
           << ", \"restarts\": " << restarts << ", \"lost\": " << (lost ? "true" : "false")
           << ", \"errors\": " << errNow << ", \"mount\": " << jsonStr(mountState)
           << ", \"mount_dy\": " << (lastVerdict.have ? fmtF(lastVerdict.med5Dy) : "null")
           << ", \"mount_dx\": " << (lastVerdict.have ? fmtF(lastVerdict.med5Dx) : "null")
           << ", \"mount_roll\": " << (lastVerdict.have ? fmtF(lastVerdict.med5Roll) : "null")
           << ", \"mount_conf\": " << (lastMountSample.ok || lastMountSample.conf != 0 ? fmtF(lastMountSample.conf) : "null")
           << ", \"mount_authoritative\": " << (mountAuth ? "true" : "false")
           << ", \"clips_kept\": " << clipsKept << ", \"clips_refused\": " << planner.refused()
           << ", \"dumps\": " << dumper.written() << ", \"degrade\": " << degrade
           << ", \"flicker_bursts\": " << flickerBursts << ", \"teleports\": " << teleports
           << ", \"slot_violations\": " << slotViolations << ", \"last_event\": " << jsonStr(ev.last()) << "}\n";
        writeAtomic(dir + "/status.json", st.str());
    }

    // ── finalise ─────────────────────────────────────────────────────────────
    ev.add("stop", lost ? "cam_lost" : g_signal ? (g_signal == SIGINT ? "SIGINT" : "SIGTERM") : "minutes");
    stageName = "stop";
    deadlineNs = monoNs() + 30'000'000'000LL;
    if (running) rig->stop();                // EOS: MKVs finalised (Argus may take ~5 s)
    det->stop();
    rig->close();
    stageName = "controls";
    exposure.close();
    focus.close();
    if (ctrlFd >= 0) ::close(ctrlFd);
    // Clips still open are written with what they have (bounded by the watchdog).
    stageName = "clips";
    if (clipsOn) {
        planner.collect(ring);
        for (ClipJob& j : planner.finished(ring.newestMono(), monoNs(), true)) {
            const int id = j.id;
            if (clipper.enqueue(std::move(j))) { ++clipsKept; ev.add("clip_kept", std::to_string(id), "at stop"); }
        }
    }
    clipper.finish();
    stageName = "dumps";
    dumper.finish();
    ass.close();
    int segments = 0;
    for (const auto& e : fs::directory_iterator(reviewDir, ec)) segments += e.path().extension() == ".mkv";
    const LaneStats s = det->stats();
    ev.add("run_end");
    ev.writer().close();
    for (CsvWriter* w : { &frames, &counters, &ctrl, &clockCsv, &imgStats, &mountCsv, &clipsIdx, &dumpsIdx, &stillsIdx })
        w->close();
    std::ostringstream extra;
    extra << ", \"processed\": " << s.processed << ", \"pulled\": " << s.pulled << ", \"gated\": " << s.gated
          << ", \"errors\": {\"map\": " << s.mapFail << ", \"size\": " << s.sizeMismatch << ", \"h2d\": " << s.h2dFail
          << ", \"pre\": " << s.preFail << ", \"trt\": " << s.trtFail << ", \"d2h\": " << s.d2hFail
          << ", \"sync\": " << s.syncFail << ", \"nonfinite\": " << s.nonFinite << "}"
          << ", \"restarts\": " << restarts << ", \"cam_lost\": " << (lost ? "true" : "false")
          << ", \"review_segments\": " << segments << ", \"clips_written\": " << clipper.written()
          << ", \"clips_refused\": " << planner.refused() << ", \"clip_overflow_frames\": " << planner.overflowFrames()
          << ", \"clip_queue_drops\": " << clipper.dropped() << ", \"clip_bytes\": " << clipper.bytes()
          << ", \"dumps_written\": " << dumper.written() << ", \"dump_queue_drops\": " << dumper.dropped()
          << ", \"dump_bytes\": " << dumper.bytes() << ", \"sig_drops\": " << sigs.dropped()
          << ", \"ring_frames\": " << rig->ringFrames() << ", \"mount\": " << jsonStr(mountState)
          << ", \"mount_shifted\": " << (lastVerdict.shifted ? "true" : "false")
          << ", \"mount_authoritative\": " << (mountAuth ? "true" : "false")
          << ", \"flicker_bursts\": " << flickerBursts << ", \"teleports\": " << teleports
          << ", \"slot_violations\": " << slotViolations << ", \"stills\": " << stillsTaken
          << ", \"degrade\": " << degrade << ", \"exit_reason\": "
          << jsonStr(lost ? "cam_lost" : g_signal ? "signal" : "minutes") << ", \"stop_hang\": false";
    writeSummary(extra.str());
    finished = true;
    watchdog.join();
    return lost ? 3 : 0;
}
