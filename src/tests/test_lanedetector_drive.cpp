// lane_drive — the in-car / bench harness of the lane test suite (plan P5a,
// ~/drive_logs/lane_test_suite_plan_20261025.md sections 3.3, 10, 12).
//
//   lane_drive --cam csi|usb --out RUN_DIR [options]       one camera per process
//
// Runs liblanedetector live on one camera and logs everything a drive needs
// to be analysed afterwards, under RUN_DIR/<cam>/:
//   frames.csv       every result: timestamps, stage timings, lanes, per-slot geometry
//   counters.csv     1 Hz: source fps, lane rate, gate/drops, errors, process RSS/threads/fds
//   camera_ctrl.csv  1 Hz: exposure / gain / focus (UGREEN; CSI: unavailable, see P0)
//   clock.csv        1 Hz: CLOCK_MONOTONIC / REALTIME / BOOTTIME triples (CAN join, steps)
//   events.csv       lifecycle and faults (start, lanes_on, cam_error, cam_restart, stall, stop, run_end)
//   review/*.mkv     review footage, 60 s segments: CSI JPEG at --review-fps (5), UGREEN MJPEG
//                    passthrough at --review-fps (10)
//   run.json         options, engine, geometry;  summary.json  totals at the end
//
// CSI: the production Camera_CSI (native 1456x1088@60, capture branch off), lane
// bin + review bin as leaky branches.  UGREEN: a pipeline of its own — MJPEG
// passthrough for review, decoded for lanes by jpegdec (--usb-decode sw, the
// default) or nvv4l2decoder (hw), a 2 Hz luma tap for v0.4's frame-rate-
// priority exposure loop; focus held at --focus (690) — both handed back at
// exit.  Buffers are restamped on arrival and decimated by a probe (no
// videorate on the lane path; see restampProbe / Decimator).
// KNOWN BROKEN (2026-10-25): --usb-decode hw delivers 13.4 Hz for 20 with a
// latency that grows ~1 s per 3 s (capture->done 17 s after a minute), also
// with enable-max-performance / disable-dpb; sw measured 20.0 Hz, p50 43 ms.
//
// The first --lanes-off-s seconds run with the lane branch closed (baseline
// for the resource deltas).  A camera error or a 10 s stall restarts the
// camera (5 tries, 2/4/8/16/30 s backoff, then cam_lost).
// SIGINT / SIGTERM: finalise (EOS → MKVs closed, CSVs flushed, controls
// handed back) and exit 0.  A finalising watchdog writes summary.json with
// stop_hang and exits 4 if that takes over 30 s.  RUN_DIR must be outside the
// repo (public).

#include "libcamera.h"
#include "libcamera_csi.h"
#include "libcamera_exposure.h"
#include "libcamera_focus.h"
#include "lane_eval_common.h"
#include "lane_test_common.h"
#include "liblanedetector.h"
#include "liblog.h"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <linux/videodev2.h>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace dashcam::camera;
using namespace dashcam::lane;
using laneeval::CsvWriter;
using laneeval::fmtF;
using laneeval::monoNs;
using laneeval::realNs;
using laneeval::bootNs;

namespace {

std::atomic<int> g_signal{0};
void onSignal(int sig) { g_signal = sig; }

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
    int stallS = 10;
};

[[noreturn]] void usage(const char* msg = nullptr) {
    if (msg) std::cerr << "lane_drive: " << msg << "\n";
    std::cerr <<
        "usage: lane_drive --cam csi|usb --out RUN_DIR [--engine E] [--crop T,B] [--refy Y]\n"
        "                  [--hz 20] [--branch-fps 20] [--rate-gate pts|legacy] [--review-fps N]\n"
        "                  [--jpeg-q 75] [--usb-dev DEV] [--usb-decode hw|sw]\n"
        "                  [--usb-exposure framerate|camera] [--target-luma 110] [--focus 690|-1]\n"
        "                  [--minutes N] [--lanes-off-s 120] [--stall-s 10]\n";
    std::exit(1);
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
            const std::string v = next();
            const auto c = v.find(',');
            if (c == std::string::npos) usage("--crop wants T,B");
            o.cropTop = std::stof(v.substr(0, c));
            o.cropBottom = std::stof(v.substr(c + 1));
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
        else if (a == "--stall-s")      o.stallS = std::stoi(next());
        else usage(("unknown option " + a).c_str());
    }
    if (o.cam != "csi" && o.cam != "usb") usage("--cam must be csi or usb");
    if (o.out.empty()) usage("--out is required");
    if (o.rateGate != "pts" && o.rateGate != "legacy") usage("--rate-gate pts|legacy");
    if (o.usbDecode != "hw" && o.usbDecode != "sw") usage("--usb-decode hw|sw");
    if (o.usbExposure != "framerate" && o.usbExposure != "camera") usage("--usb-exposure framerate|camera");
    // Per-camera defaults.  CSI: the libconfig <Detection> crops (rows
    // [544, 864) of 1088) until the mount calibration (12.1) sets them.
    // UGREEN: top half only — the CSI bottom crop cuts the UGREEN's nearest
    // lane markings (P1 finding: 0/298 night frames with lanes).
    if (o.cropTop < 0)    o.cropTop    = 0.5f;
    if (o.cropBottom < 0) o.cropBottom = o.cam == "csi" ? 0.2059f : 0.0f;
    if (o.reviewFps < 0)  o.reviewFps  = o.cam == "csi" ? 5 : 10;
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

// A bin from @p desc with a "sink" ghost pad on its first element.  Not
// gst_parse_bin_from_description(ghost_unlinked=TRUE): with automatic
// ghosting, splitmuxsink fails its READY transition ("Failed to link muxer and
// sink - splitmuxsink will not work", seen on this GStreamer 1.20).
GstElement* reviewBin(const std::string& desc, std::string& why) {
    GError* err = nullptr;
    GstElement* bin = gst_parse_bin_from_description(desc.c_str(), FALSE, &err);
    if (!bin || err) {
        why = std::string("review bin: ") + (err ? err->message : "?");
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
        why = "review bin: no sink pad to ghost";
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
// instead of videorate on the UGREEN lane path: videorate stamps output on a
// timeline of its own, which on this live pipeline ran ~1 s ahead of the
// source after start-up (capture->done measured at -0.9 s; 22-25 Hz for 20).
struct Decimator {
    GstClockTime interval = 0;
    GstClockTime next = GST_CLOCK_TIME_NONE;
    GstClockTime lastPts = GST_CLOCK_TIME_NONE;
};
GstPadProbeReturn decimateProbe(GstPad*, GstPadProbeInfo* info, gpointer user) {
    auto* d = static_cast<Decimator*>(user);
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf || d->interval == 0) return GST_PAD_PROBE_OK;
    const GstClockTime pts = GST_BUFFER_PTS(buf);
    if (!GST_CLOCK_TIME_IS_VALID(pts)) return GST_PAD_PROBE_OK;
    const GstClockTime frame = GST_CLOCK_TIME_IS_VALID(d->lastPts) && pts > d->lastPts ? pts - d->lastPts : 0;
    d->lastPts = pts;
    if (!GST_CLOCK_TIME_IS_VALID(d->next) || pts + d->interval < d->next || pts > d->next + d->interval) {
        d->next = pts + d->interval;
        return GST_PAD_PROBE_OK;
    }
    if (pts + frame / 2 > d->next) {
        d->next += d->interval;
        return GST_PAD_PROBE_OK;
    }
    return GST_PAD_PROBE_DROP;
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

// ─── events ──────────────────────────────────────────────────────────────────

class Events {
public:
    bool open(const std::string& path, const std::string& cam) {
        cam_ = cam;
        return w_.open(path, "mono_ns,realtime_ns,cam,type,value,detail");
    }
    void add(const std::string& type, const std::string& value = "", const std::string& detail = "") {
        std::string d = detail;
        for (char& c : d) if (c == ',' || c == '\n') c = ';';
        w_.row(std::to_string(monoNs()) + "," + std::to_string(realNs()) + "," + cam_ + "," + type + "," +
               value + "," + d);
        std::fprintf(stderr, "[lane_drive %s] %s %s %s\n", cam_.c_str(), type.c_str(), value.c_str(), d.c_str());
    }
    CsvWriter& writer() { return w_; }
private:
    std::string cam_;
    CsvWriter w_;
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
    virtual void close() {}                           ///< release the device for good
    virtual double nominalFps() const = 0;
    virtual bool lumaSample(float& /*luma*/) { return false; }
    uint64_t offered() const { return offered_.load(std::memory_order_relaxed); }
protected:
    std::atomic<uint64_t> offered_{0};
};

class CsiRig : public Rig {
public:
    CsiRig(const Options& o, const std::string& reviewDir, dashcam::log::LogCallback log)
        : o_(o), reviewDir_(reviewDir), log_(std::move(log)) {}
    bool init(std::string& why, uint32_t& w, uint32_t& h) {
        std::vector<cameraInfo> cams;
        if (getCameraList(cams) != ERROR_CODE::NONE) { why = "getCameraList failed"; return false; }
        for (const auto& c : cams)
            if (c.type == CAMERA_TYPE::CSI && isImx296(c) && !c.videoFormats.empty()) { info_ = c; found_ = true; break; }
        if (!found_) { why = "no IMX296 CSI camera"; return false; }
        w = info_.videoFormats[0].width;
        h = info_.videoFormats[0].height;
        fps_ = info_.videoFormats[0].frameRate;
        cam_ = std::make_unique<Camera_CSI>(info_);
        cam_->setLogCallback(log_);
        cam_->open();
        return true;
    }
    bool start(GstElement* laneBin, bool lanesOn, int part, std::string& why) override {
        const std::string review =
            "videorate drop-only=true max-rate=" + std::to_string(o_.reviewFps) +
            " ! nvjpegenc quality=" + std::to_string(o_.jpegQ) +
            " ! splitmuxsink muxer-factory=matroskamux max-size-time=60000000000 location=" +
            reviewDir_ + "/csi_p" + std::to_string(part) + "_%05d.mkv";
        GstElement* rbin = reviewBin(review, why);
        if (!rbin) {
            if (laneBin) gst_object_unref(gst_object_ref_sink(laneBin));
            return false;
        }
        cam_->addBranch("lanes", laneBin, /*leaky=*/true, lanesOn);
        cam_->addBranch("review", rbin, /*leaky=*/true, true);
        cam_->setCameraVideoFormat(0);
        cam_->start();
        cameraStatus st;
        cam_->getCameraStatus(st);
        if (st.status != CAMERA_STATUS::RUNNING) { why = "Camera_CSI did not reach RUNNING"; return false; }
        cam_->setCaptureEnabled(false);              // its CPU BGR conversion is not ours
        addCountProbe(cam_->getTee(), "sink", &offered_);
        return true;
    }
    void stop() override { if (cam_) cam_->stop(); }
    void close() override { if (cam_) cam_->close(); }
    bool failed(std::string& why) override {
        cameraStatus st;
        cam_->getCameraStatus(st);
        if (st.status == CAMERA_STATUS::ERROR) { why = "Camera_CSI status ERROR"; return true; }
        return false;
    }
    void setLanes(bool on) override { cam_->setBranchEnabled("lanes", on); }
    double nominalFps() const override { return fps_; }
private:
    Options o_;
    std::string reviewDir_;
    dashcam::log::LogCallback log_;
    cameraInfo info_{};
    bool found_ = false;
    double fps_ = 60;
    std::unique_ptr<Camera_CSI> cam_;
};

class UsbRig : public Rig {
public:
    UsbRig(const Options& o, const std::string& reviewDir) : o_(o), reviewDir_(reviewDir) {}
    ~UsbRig() override { stop(); }
    bool start(GstElement* laneBin, bool lanesOn, int part, std::string& why) override {
        const std::string decode = o_.usbDecode == "hw"
            ? "jpegparse ! nvv4l2decoder mjpeg=1 enable-max-performance=true disable-dpb=true" : "jpegdec";
        const std::string desc =
            "v4l2src name=src device=" + o_.usbDev + " ! image/jpeg,width=" + std::to_string(o_.usbW) +
            ",height=" + std::to_string(o_.usbH) + ",framerate=" + std::to_string(o_.usbFps) + "/1 ! tee name=t "
            "t. ! queue leaky=downstream max-size-buffers=8 max-size-bytes=0 max-size-time=0 "
            "! videorate drop-only=true max-rate=" + std::to_string(o_.reviewFps) +
            " ! splitmuxsink muxer-factory=matroskamux max-size-time=60000000000 location=" + reviewDir_ +
            "/usb_p" + std::to_string(part) + "_%05d.mkv "
            "t. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            "! valve name=lanevalve drop=" + std::string(lanesOn ? "false" : "true") +
            " ! identity name=decim ! " + decode + " ! identity name=lanetail "
            "t. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            "! videorate drop-only=true max-rate=2 ! jpegdec "
            "! appsink name=luma max-buffers=1 drop=true sync=false emit-signals=false";
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
        valve_ = gst_bin_get_by_name(GST_BIN(pipe_), "lanevalve");
        luma_  = gst_bin_get_by_name(GST_BIN(pipe_), "luma");
        GstElement* tee = gst_bin_get_by_name(GST_BIN(pipe_), "t");
        addCountProbe(tee, "sink", &offered_);
        gst_object_unref(tee);
        decim_ = Decimator{};
        decim_.interval = o_.branchFps > 0 ? GST_SECOND / o_.branchFps : 0;
        GstElement* dec = gst_bin_get_by_name(GST_BIN(pipe_), "decim");
        if (GstPad* dp = gst_element_get_static_pad(dec, "src")) {
            gst_pad_add_probe(dp, GST_PAD_PROBE_TYPE_BUFFER, decimateProbe, &decim_, nullptr);
            gst_object_unref(dp);
        }
        gst_object_unref(dec);
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
        setLanes(true);                       // a closed valve would swallow the EOS
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
    double nominalFps() const override { return o_.usbFps; }
    // Mean of the Y plane of the latest 2 Hz luma frame, subsampled.
    bool lumaSample(float& luma) override {
        if (!luma_) return false;
        GstSample* s = gst_app_sink_try_pull_sample(GST_APP_SINK(luma_), 0);
        if (!s) return false;
        GstVideoInfo info;
        GstVideoFrame vf;
        bool ok = false;
        if (gst_video_info_from_caps(&info, gst_sample_get_caps(s)) && GST_VIDEO_INFO_IS_YUV(&info) &&
            gst_video_frame_map(&vf, &info, gst_sample_get_buffer(s), GST_MAP_READ)) {
            const auto* y = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&vf, 0));
            const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vf, 0);
            const int w = GST_VIDEO_FRAME_WIDTH(&vf), h = GST_VIDEO_FRAME_HEIGHT(&vf);
            uint64_t sum = 0, n = 0;
            for (int r = 0; r < h; r += 4)
                for (int c = 0; c < w; c += 4) { sum += y[static_cast<size_t>(r) * stride + c]; ++n; }
            gst_video_frame_unmap(&vf);
            if (n) { luma = static_cast<float>(sum) / n; ok = true; }
        }
        gst_sample_unref(s);
        return ok;
    }
private:
    void teardown() {
        if (!pipe_) return;
        gst_element_set_state(pipe_, GST_STATE_NULL);
        if (valve_) { gst_object_unref(valve_); valve_ = nullptr; }
        if (luma_)  { gst_object_unref(luma_);  luma_  = nullptr; }
        gst_object_unref(pipe_);
        pipe_ = nullptr;
    }
    Options o_;
    std::string reviewDir_;
    GstElement* pipe_  = nullptr;
    GstElement* valve_ = nullptr;
    GstElement* luma_  = nullptr;
    Decimator   decim_;
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
    return h;
}

std::string frameRow(const std::string& cam, const LaneResult& r) {
    const LaneTiming& t = r.timing;
    std::string s;
    s.reserve(512);
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
    return s;
}

} // namespace

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    gst_init(&argc, &argv);
    const Options o = parseOptions(argc, argv);
    namespace fs = std::filesystem;

    if (lanetest::insideRepo(o.out)) {
        std::cerr << "lane_drive: --out " << o.out << " is inside the repo; refusing\n";
        return 1;
    }
    const std::string dir = o.out + "/" + o.cam;
    const std::string reviewDir = dir + "/review";
    std::error_code ec;
    fs::create_directories(reviewDir, ec);
    if (ec) { std::cerr << "lane_drive: cannot create " << reviewDir << ": " << ec.message() << "\n"; return 1; }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    Events ev;
    CsvWriter frames(50000), counters, ctrl, clockCsv;
    const bool csvOk = ev.open(dir + "/events.csv", o.cam) &&
        frames.open(dir + "/frames.csv", framesHeader()) &&
        counters.open(dir + "/counters.csv",
            "mono_ns,cam_state,lanes_on,offered,pulled,gated,processed,map_fail,size_mismatch,h2d_fail,"
            "pre_fail,trt_fail,d2h_fail,sync_fail,nonfinite,eff_hz,src_fps,restarts,frames_csv_drops,"
            "rss_kb,threads,fds") &&
        ctrl.open(dir + "/camera_ctrl.csv",
            "mono_ns,exposure,exposure_unit,gain,gain_unit,auto_exposure,focus,focus_auto,src_fps,luma,"
            "night_mode,ctrl_source") &&
        clockCsv.open(dir + "/clock.csv", "mono_ns,realtime_ns,boottime_ns,wall_minus_mono_ns,step_ms");
    if (!csvOk) { std::cerr << "lane_drive: cannot open CSV files in " << dir << "\n"; return 1; }

    const int64_t startMono = monoNs(), startReal = realNs();
    ev.add("start", o.cam);

    // Watchdog: a start or stop that hangs must still leave usable data.
    std::atomic<int64_t> deadlineNs{monoNs() + 90'000'000'000LL};   // start budget (engine load + Argus)
    std::atomic<const char*> stageName{"start"};
    std::atomic<bool> finished{false};
    auto writeSummary = [&](const std::string& extra) {
        std::ofstream s(dir + "/summary.json");
        s << "{\"cam\": " << jsonStr(o.cam) << ", \"start_mono_ns\": " << startMono
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
                frames.close(); counters.close(); ctrl.close(); clockCsv.close();
                writeSummary(std::string(", \"stop_hang\": true, \"hung_in\": ") + jsonStr(stageName.load()));
                std::_Exit(4);
            }
        }
    });

    auto logCb = [&ev](dashcam::log::LogLevel lvl, const std::string& msg) {
        if (lvl == dashcam::log::LogLevel::ERROR || lvl == dashcam::log::LogLevel::WARN)
            ev.add(lvl == dashcam::log::LogLevel::ERROR ? "lib_error" : "lib_warn", "", msg);
    };

    // ── camera + detector ────────────────────────────────────────────────────
    std::unique_ptr<Rig> rig;
    uint32_t srcW = 0, srcH = 0;
    std::string why;
    if (o.cam == "csi") {
        auto c = std::make_unique<CsiRig>(o, reviewDir, logCb);
        if (!c->init(why, srcW, srcH)) { ev.add("cam_error", "init", why); finished = true; watchdog.join(); return 2; }
        rig = std::move(c);
    } else {
        srcW = static_cast<uint32_t>(o.usbW);
        srcH = static_cast<uint32_t>(o.usbH);
        rig = std::make_unique<UsbRig>(o, reviewDir);
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
    det->setResultSink([&frames, cam = o.cam](const LaneResult& r) { frames.row(frameRow(cam, r)); });

    {   // run.json
        const LaneGeometry& g = det->geometry();
        std::ofstream rj(dir + "/run.json");
        rj << "{\n \"cam\": " << jsonStr(o.cam) << ",\n \"engine\": " << jsonStr(o.engine)
           << ",\n \"engine_bytes\": " << fs::file_size(o.engine, ec)
           << ",\n \"src\": [" << srcW << ", " << srcH << "],\n \"crop\": [" << o.cropTop << ", " << o.cropBottom
           << "],\n \"ref_y\": " << o.refY << ",\n \"target_hz\": " << o.hz << ",\n \"branch_fps\": " << o.branchFps
           << ",\n \"rate_gate\": " << jsonStr(o.rateGate) << ",\n \"review_fps\": " << o.reviewFps
           << ",\n \"jpeg_q\": " << o.jpegQ << ",\n \"usb_decode\": " << jsonStr(o.usbDecode)
           << ",\n \"usb_exposure\": " << jsonStr(o.usbExposure) << ",\n \"target_luma\": " << o.targetLuma
           << ",\n \"focus\": " << o.focus << ",\n \"lanes_off_s\": " << o.lanesOffS
           << ",\n \"capture_branch_disabled\": " << (o.cam == "csi" ? "true" : "null")
           << ",\n \"geometry\": {\"crop_top_px\": " << g.cropTopPx << ", \"crop_bottom_px\": " << g.cropBottomPx
           << ", \"branch\": [" << g.branchW << ", " << g.branchH << "], \"eff_ratio\": " << fmtF(g.effRatio)
           << ", \"pre_y_scale\": " << fmtF(g.preYScale) << ", \"pre_y_off\": " << fmtF(g.preYOff)
           << ", \"implied_h\": " << fmtF(g.impliedH) << ", \"implied_top_src\": " << fmtF(g.impliedTopSrc)
           << ", \"ref_row_src\": " << fmtF(g.refRowSrc) << "},\n \"start_mono_ns\": " << startMono
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

    // ── run ──────────────────────────────────────────────────────────────────
    bool lanesOn = o.lanesOffS <= 0;
    int part = 0, restarts = 0, failures = 0;
    bool running = false, lost = false;
    auto startCamera = [&]() -> bool {
        stageName = "start";
        deadlineNs = monoNs() + 90'000'000'000LL;
        GstElement* bin = det->createBin();
        if (!bin) { ev.add("cam_error", "createBin"); return false; }
        std::string w;
        const bool ok = rig->start(bin, lanesOn, part, w);
        if (!ok) ev.add("cam_error", "start", w);
        else det->start();
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

    const int64_t endNs = o.minutes > 0 ? monoNs() + static_cast<int64_t>(o.minutes * 60e9) : INT64_MAX;
    const int64_t lanesOnAt = startMono + static_cast<int64_t>(o.lanesOffS) * 1'000'000'000LL;
    int64_t lastTick = monoNs(), lastWallMinusMono = realNs() - monoNs();
    uint64_t lastProcessed = 0, lastOffered = 0, lastPulled = 0;
    int64_t lastPullProgressNs = monoNs(), nextRetryNs = 0;
    std::deque<std::pair<double, uint64_t>> fpsWin;
    float lastLuma = NAN;
    const int backoff[] = { 2, 4, 8, 16, 30 };

    while (!g_signal && monoNs() < endNs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const int64_t now = monoNs();

        // Exposure loop (~2 Hz luma tap), measured fps from the tee counter.
        float luma = 0;
        if (running && rig->lumaSample(luma)) {
            lastLuma = luma;
            const double t = now / 1e9;
            fpsWin.emplace_back(t, rig->offered());
            while (fpsWin.size() > 2 && t - fpsWin.front().first > 2.5) fpsWin.pop_front();
            const double span = t - fpsWin.front().first;
            const float fps = span >= 1.0 ? static_cast<float>((fpsWin.back().second - fpsWin.front().second) / span) : NAN;
            if (exposure.isOpen()) exposure.onLuma(luma, fps, t);
        }

        if (!lanesOn && now >= lanesOnAt) {
            lanesOn = true;
            if (running) rig->setLanes(true);
            lastPullProgressNs = now;          // the stall clock starts with the lanes
            ev.add("lanes_on");
        }

        // Health: camera error, or no frame reaching the detector for stallS.
        const uint64_t pulled = det->stats().pulled;
        if (pulled != lastPulled) { lastPulled = pulled; lastPullProgressNs = now; }
        if (running) {
            std::string w;
            const bool stalled = lanesOn && now - lastPullProgressNs > static_cast<int64_t>(o.stallS) * 1'000'000'000LL;
            if (rig->failed(w) || stalled) {
                ev.add(stalled ? "stall" : "cam_error", "", stalled ? "no frame reached the detector" : w);
                stageName = "restart stop";
                deadlineNs = now + 30'000'000'000LL;
                rig->stop();
                det->stop();
                deadlineNs = INT64_MAX;
                running = false;
                nextRetryNs = now + backoff[std::min(failures, 4)] * 1'000'000'000LL;
            }
        } else if (!lost && now >= nextRetryNs) {
            if (failures >= 5) {
                lost = true;
                ev.add("cam_lost", std::to_string(failures));
            } else {
                ++failures;
                ++restarts;
                ++part;
                running = startCamera();
                ev.add("cam_restart", std::to_string(restarts), running ? "ok" : "failed");
                if (running) { failures = 0; lastPullProgressNs = monoNs(); }
                else nextRetryNs = monoNs() + backoff[std::min(failures, 4)] * 1'000'000'000LL;
            }
        }

        if (now - lastTick < 1'000'000'000LL) continue;
        // ── 1 Hz rows ────────────────────────────────────────────────────────
        const double dt = (now - lastTick) / 1e9;
        lastTick = now;
        const LaneStats s = det->stats();
        const uint64_t offered = rig->offered();
        const std::string state = lost ? "LOST" : running ? "RUNNING" : "RESTARTING";
        counters.row(std::to_string(now) + "," + state + "," + std::to_string(lanesOn) + "," +
                     std::to_string(offered) + "," + std::to_string(s.pulled) + "," + std::to_string(s.gated) + "," +
                     std::to_string(s.processed) + "," + std::to_string(s.mapFail) + "," +
                     std::to_string(s.sizeMismatch) + "," + std::to_string(s.h2dFail) + "," +
                     std::to_string(s.preFail) + "," + std::to_string(s.trtFail) + "," +
                     std::to_string(s.d2hFail) + "," + std::to_string(s.syncFail) + "," +
                     std::to_string(s.nonFinite) + "," + fmtF((s.processed - lastProcessed) / dt) + "," +
                     fmtF((offered - lastOffered) / dt) + "," + std::to_string(restarts) + "," +
                     std::to_string(frames.drops()) + "," + std::to_string(procStatusKb("VmRSS:")) + "," +
                     std::to_string(countDir("/proc/self/task")) + "," + std::to_string(countDir("/proc/self/fd")));
        const double srcFps = (offered - lastOffered) / dt;
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
            ctrl.row(std::to_string(now) + ",,lines,,0.1dB,,,," + fmtF(srcFps) + ",,,unavailable");
        }

        const int64_t mono = monoNs(), real = realNs(), wmm = real - mono;   // sampled together
        const double stepMs = (wmm - lastWallMinusMono) / 1e6;
        clockCsv.row(std::to_string(mono) + "," + std::to_string(real) + "," + std::to_string(bootNs()) + "," +
                     std::to_string(wmm) + "," + fmtF(stepMs));
        if (std::fabs(stepMs) > 50) ev.add("clock_step", fmtF(stepMs));
        lastWallMinusMono = wmm;
    }

    // ── finalise ─────────────────────────────────────────────────────────────
    ev.add("stop", g_signal ? (g_signal == SIGINT ? "SIGINT" : "SIGTERM") : "minutes");
    stageName = "stop";
    deadlineNs = monoNs() + 30'000'000'000LL;
    if (running) rig->stop();                // EOS: MKVs finalised (Argus may take ~5 s)
    det->stop();
    rig->close();
    stageName = "controls";
    exposure.close();
    focus.close();
    if (ctrlFd >= 0) ::close(ctrlFd);
    int segments = 0;
    for (const auto& e : fs::directory_iterator(reviewDir, ec)) segments += e.path().extension() == ".mkv";
    const LaneStats s = det->stats();
    ev.add("run_end");
    ev.writer().close();
    frames.close();
    counters.close();
    ctrl.close();
    clockCsv.close();
    std::ostringstream extra;
    extra << ", \"processed\": " << s.processed << ", \"pulled\": " << s.pulled << ", \"gated\": " << s.gated
          << ", \"errors\": {\"map\": " << s.mapFail << ", \"size\": " << s.sizeMismatch << ", \"h2d\": " << s.h2dFail
          << ", \"pre\": " << s.preFail << ", \"trt\": " << s.trtFail << ", \"d2h\": " << s.d2hFail
          << ", \"sync\": " << s.syncFail << ", \"nonfinite\": " << s.nonFinite << "}"
          << ", \"restarts\": " << restarts << ", \"cam_lost\": " << (lost ? "true" : "false")
          << ", \"review_segments\": " << segments << ", \"exit_reason\": "
          << jsonStr(g_signal ? "signal" : "minutes") << ", \"stop_hang\": false";
    writeSummary(extra.str());
    finished = true;
    watchdog.join();
    return 0;
}
