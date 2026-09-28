// camera_gst_test — Camera_GST (the GStreamer base of Camera_USB / Camera_CSI)
// without a camera: videotestsrc stands in for v4l2src / nvarguscamerasrc,
// behind the same captureBranch() both drivers end their pipelines with.
//
// Covers the lifecycle (open/start/capture/stop/close, restart, recovery after
// ERROR), BGR capture from an odd-width source (row stride ≠ width × 3), the
// capture valve, branches (linking, runtime enable/disable, misuse), attributes
// (queued before start, applied while running, unknown alias, bad value, a
// property the element lacks, a value outside its type or range, V4L2
// controls through v4l2src's extra-controls; every entry of
// config/camera_attributes.xml against the real element),
// start failures and runtime bus errors (with the element and GStreamer's
// detail in the log), and the USB / CSI pipeline strings.
//
// Needs only GStreamer core + base plugins (videotestsrc, videoconvert).
// Runs anywhere: cloud sandbox, dev container, Jetson.
//
// Usage: camera_gst_test

#include "libcamera_csi.h"
#include "libcamera_gst.h"
#include "libcamera_usb.h"

#include <linux/videodev2.h>

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
using dashcam::log::LogLevel;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

// ─── fixtures ────────────────────────────────────────────────────────────────

/// Collects everything the camera logs, for checks on the diagnostics.
struct LogSink {
    std::mutex               m;
    std::vector<std::string> lines;
    dashcam::log::LogCallback callback() {
        return [this](LogLevel lvl, const std::string& msg) {
            std::lock_guard<std::mutex> lk(m);
            lines.push_back(std::string(lvl == LogLevel::ERROR ? "E " :
                                        lvl == LogLevel::WARN  ? "W " : "I ") + msg);
        };
    }
    bool has(const std::string& needle) {
        std::lock_guard<std::mutex> lk(m);
        return std::any_of(lines.begin(), lines.end(),
                           [&](const std::string& l) { return l.find(needle) != std::string::npos; });
    }
    void clear() {
        std::lock_guard<std::mutex> lk(m);
        lines.clear();
    }
};

constexpr uint32_t kW = 321;   // odd: the BGR row stride is padded to 964, not 963
constexpr uint32_t kH = 241;

static cameraInfo testInfo() {
    cameraInfo ci;
    ci.type     = CAMERA_TYPE::USB;
    ci.address  = "videotestsrc";
    ci.deviceId = 0;
    ci.videoFormats.push_back({kW, kH, 30.0f, V4L2_PIX_FMT_YUV420, "I420"});
    return ci;
}

/// Camera_GST over videotestsrc.  @c description replaces the whole pipeline
/// (start-failure tests); otherwise the source is followed by the drivers'
/// own captureBranch().
class TestCamera : public Camera_GST {
public:
    explicit TestCamera(const cameraInfo& ci = testInfo()) : Camera_GST(ci) {}
    ~TestCamera() override = default;

    using Camera_GST::applyGstProperty;
    using Camera_GST::setExtraControl;

    std::string description;

    CAMERA_STATUS state() const {
        cameraStatus s;
        getCameraStatus(s);
        return s.status;
    }
    ERROR_CODE error() const {
        cameraStatus s;
        getCameraStatus(s);
        return s.currentError;
    }
    uint64_t frames() const {
        cameraStatus s;
        getCameraStatus(s);
        return s.frameCount;
    }
    /// Posts an ERROR from the source element, as a failing camera would.
    void failSource(const char* debug) {
        std::lock_guard<std::mutex> lk(stateMutex_);
        GError* err = g_error_new_literal(GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_READ,
                                          "simulated device failure");
        gst_element_post_message(camera_src_,
                                 gst_message_new_error(GST_OBJECT(camera_src_), err, debug));
        g_error_free(err);
    }
    /// Posts a WARNING from the source element.
    void warnSource(const char* text) {
        std::lock_guard<std::mutex> lk(stateMutex_);
        GError* err = g_error_new_literal(GST_CORE_ERROR, GST_CORE_ERROR_FAILED, text);
        gst_element_post_message(camera_src_,
                                 gst_message_new_warning(GST_OBJECT(camera_src_), err, nullptr));
        g_error_free(err);
    }
    int sourceInt(const char* property) {
        std::lock_guard<std::mutex> lk(stateMutex_);
        gint v = -1;
        if (camera_src_) g_object_get(camera_src_, property, &v, nullptr);
        return v;
    }

protected:
    ERROR_CODE pipelineError() const override { return ERROR_CODE::USB_PIPELINE_ERROR; }
    const char* cameraTypeTag() const override { return "USB"; }
    std::string buildPipelineString(const cameraVideoFormat& fmt,
                                    uint32_t frNum, uint32_t frDen) const override {
        if (!description.empty()) return description;
        return "videotestsrc name=camerasrc is-live=true"
               " ! video/x-raw, format=(string)I420, width=" + std::to_string(fmt.width) +
               ", height=" + std::to_string(fmt.height) +
               ", framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen) +
               captureBranch("videoconvert ! video/x-raw, format=(string)BGR");
    }
};

/// The drivers' pipeline strings, for the contract checks.
struct UsbProbe : Camera_USB {
    using Camera_USB::Camera_USB;
    std::string pipeline(const cameraVideoFormat& f) const {
        uint32_t n, d;
        computeFpsRational(f.frameRate, n, d);
        return buildPipelineString(f, n, d);
    }
};
struct CsiProbe : Camera_CSI {
    using Camera_CSI::Camera_CSI;
    std::string pipeline(const cameraVideoFormat& f) const {
        uint32_t n, d;
        computeFpsRational(f.frameRate, n, d);
        return buildPipelineString(f, n, d);
    }
};

static AttributeDictionary testDictionary() {
    AttributeDictionary d;
    d.entries.push_back({"pattern", "USB", AttributeValueType::Int, {"pattern", "Test Pattern"}});
    d.entries.push_back({"horizontal-speed", "any", AttributeValueType::Int, {"speed"}});
    d.entries.push_back({"is-live", "CSI", AttributeValueType::Bool, {"csi only"}});
    d.entries.push_back({"no-such-property", "USB", AttributeValueType::Int, {"typo"}});
    d.entries.push_back({"horizontal-speed", "USB", AttributeValueType::String, {"speed as text"}});
    d.entries.push_back({"sharpness", "USB", AttributeValueType::V4l2Control, {"sharpness"}});
    return d;
}

static PipelineParams fastParams() {
    PipelineParams p;
    p.captureTimeoutMs     = 400;   // disabled-capture checks wait this long
    p.stateChangeTimeoutMs = 3000;
    p.eosTimeoutMs         = 2000;
    return p;
}

/// Pulls until a frame arrives or @p tries run out; bytes written.
static uint32_t pull(TestCamera& cam, std::vector<uint8_t>& buf, int tries = 10) {
    uint32_t written = 0;
    for (int i = 0; i < tries && written == 0; ++i)
        cam.captureFrame(buf.data(), static_cast<uint32_t>(buf.size()), written);
    return written;
}

static double meanOf(const std::vector<uint8_t>& buf, size_t n) {
    double sum = 0;
    for (size_t i = 0; i < n; ++i) sum += buf[i];
    return n ? sum / n : 0.0;
}

// ─── tests ───────────────────────────────────────────────────────────────────

static void testLifecycleAndCapture() {
    std::printf("\n--- lifecycle and capture ---\n");
    LogSink log;
    TestCamera cam;
    cam.setLogCallback(log.callback());
    cam.setPipelineParams(fastParams());

    cam.start();
    check(cam.error() == ERROR_CODE::CAMERA_NOT_OPEN, "start() before open() -> CAMERA_NOT_OPEN");

    cam.open();
    check(cam.state() == CAMERA_STATUS::OPEN && cam.error() == ERROR_CODE::NONE, "open() -> OPEN");
    cam.open();
    check(cam.error() == ERROR_CODE::CAMERA_ALREADY_OPEN, "second open() -> CAMERA_ALREADY_OPEN");

    cam.setCameraVideoFormat(5);
    check(cam.error() == ERROR_CODE::UNSUPPORTED_FORMAT, "format index out of range -> UNSUPPORTED_FORMAT");
    cam.setCameraVideoFormat(0);

    cam.start();
    check(cam.state() == CAMERA_STATUS::RUNNING && cam.error() == ERROR_CODE::NONE, "start() -> RUNNING");
    cam.start();
    check(cam.error() == ERROR_CODE::CAMERA_ALREADY_RUNNING, "second start() -> CAMERA_ALREADY_RUNNING");

    const size_t frameBytes = size_t(kW) * kH * 3;
    std::vector<uint8_t> buf(frameBytes);
    check(pull(cam, buf) == frameBytes, "frame of width x height x 3 bytes (odd width, padded stride)");
    check(cam.frames() >= 1, "frameCount counts captured frames");

    // SMPTE bars are constant down each column in the top part of the frame:
    // a stride error would shear row 40 against row 0.
    const size_t row = size_t(kW) * 3;
    check(std::equal(buf.begin(), buf.begin() + row, buf.begin() + 40 * row),
          "rows are copied at the source stride (row 40 == row 0 in the bars)");

    std::vector<uint8_t> small(frameBytes - 1);
    uint32_t written = 1;
    cam.captureFrame(small.data(), static_cast<uint32_t>(small.size()), written);
    check(written == 0, "a buffer one byte too small gets no frame");

    const auto t0 = std::chrono::steady_clock::now();
    cam.stop();
    const double stopSec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    check(cam.state() == CAMERA_STATUS::OPEN, "stop() -> OPEN");
    check(stopSec < 2.5, "stop() drains and finishes within the EOS window (" +
                         std::to_string(stopSec).substr(0, 4) + " s)");
    cam.captureFrame(buf.data(), static_cast<uint32_t>(buf.size()), written);
    check(written == 0, "captureFrame() after stop() returns nothing");

    cam.start();
    check(cam.state() == CAMERA_STATUS::RUNNING && pull(cam, buf) == frameBytes,
          "restart after stop() captures again");
    cam.close();
    check(cam.state() == CAMERA_STATUS::CLOSED, "close() while running -> CLOSED");
    cam.close();
    check(cam.error() == ERROR_CODE::CAMERA_ALREADY_CLOSED, "second close() -> CAMERA_ALREADY_CLOSED");
}

static void testCaptureValve() {
    std::printf("\n--- capture valve ---\n");
    TestCamera cam;
    cam.setPipelineParams(fastParams());
    cam.setCaptureEnabled(false);
    check(cam.error() == ERROR_CODE::CAMERA_NOT_OPEN, "setCaptureEnabled() before start() -> CAMERA_NOT_OPEN");

    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    std::vector<uint8_t> buf(size_t(kW) * kH * 3);
    check(pull(cam, buf) == buf.size(), "capture enabled at start");

    cam.setCaptureEnabled(false);
    check(cam.error() == ERROR_CODE::NONE, "setCaptureEnabled(false) accepted");
    // Frames already queued past the valve may still arrive; after those, none.
    pull(cam, buf, 3);
    uint32_t written = 1;
    cam.captureFrame(buf.data(), static_cast<uint32_t>(buf.size()), written);
    check(written == 0, "no frames while capture is disabled");

    cam.setCaptureEnabled(true);
    check(pull(cam, buf) == buf.size(), "frames again after re-enabling");
    cam.close();
}

static std::atomic<int> g_branchBuffers{0};
static void onBranchBuffer(GstElement*, GstBuffer*, GstPad*, gpointer) { ++g_branchBuffers; }

static GstElement* countingBranch() {
    GError* err = nullptr;
    GstElement* bin = gst_parse_bin_from_description(
        "fakesink name=branchsink sync=false signal-handoffs=true", TRUE, &err);
    if (err) g_error_free(err);
    if (!bin) return nullptr;
    GstElement* sink = gst_bin_get_by_name(GST_BIN(bin), "branchsink");
    g_signal_connect(sink, "handoff", G_CALLBACK(onBranchBuffer), nullptr);
    gst_object_unref(sink);
    return bin;
}

static void testBranches() {
    std::printf("\n--- branches ---\n");
    gst_init(nullptr, nullptr);   // countingBranch() needs GStreamer before open()
    LogSink log;
    TestCamera cam;
    cam.setLogCallback(log.callback());
    cam.setPipelineParams(fastParams());

    cam.addBranch("", countingBranch());
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE, "addBranch() with an empty name refused");
    cam.addBranch("x", nullptr);
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE, "addBranch() with a null bin refused");

    cam.addBranch("rec", countingBranch(), false, true);
    cam.addBranch("paused", countingBranch(), true, false);   // leaky, starts disabled
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    check(cam.state() == CAMERA_STATUS::RUNNING, "start() links both branches");
    check(log.has("branch linked: rec") && log.has("branch linked: paused"), "both branches logged as linked");

    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const int running = g_branchBuffers.load();
    check(running > 3, "the enabled branch receives frames (" + std::to_string(running) + ")");

    cam.setBranchEnabled("rec", false);
    check(cam.error() == ERROR_CODE::NONE, "setBranchEnabled(rec, false) accepted");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));   // let in-flight frames land
    const int stopped = g_branchBuffers.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    check(g_branchBuffers.load() == stopped, "no frames reach either branch while both are disabled");

    cam.setBranchEnabled("paused", true);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    check(g_branchBuffers.load() > stopped, "enabling the branch that started disabled lets frames through");

    cam.setBranchEnabled("nosuch", true);
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE, "setBranchEnabled() on an unknown branch -> INVALID_ATTRIBUTE");
    GstElement* late = countingBranch();
    GstElement* lateRef = GST_ELEMENT(gst_object_ref(late));   // watch it being released
    cam.addBranch("late", late);
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE && log.has("camera already running"),
          "addBranch() while running refused, and logged");
    check(GST_OBJECT_REFCOUNT_VALUE(lateRef) == 1, "the refused bin was released (only our ref is left)");
    gst_object_unref(lateRef);
    cam.close();
}

static void testAttributes() {
    std::printf("\n--- attributes ---\n");
    TestCamera cam;
    cam.setAttributeDictionary(testDictionary());
    cam.setPipelineParams(fastParams());
    cam.open();
    cam.setCameraVideoFormat(0);

    cam.setCameraAttribute("TEST PATTERN", "3");   // white, queued (case-insensitive alias)
    check(cam.error() == ERROR_CODE::NONE, "attribute before start() is queued");
    cam.start();
    check(cam.sourceInt("pattern") == 3, "queued attribute applied at start()");
    std::vector<uint8_t> buf(size_t(kW) * kH * 3);
    check(pull(cam, buf) == buf.size() && meanOf(buf, buf.size()) > 240.0,
          "the first frames already show it (white)");

    cam.setCameraAttribute("pattern", "2");        // black, applied live
    check(cam.error() == ERROR_CODE::NONE && cam.sourceInt("pattern") == 2, "attribute applied while running");
    double mean = 255.0;
    for (int i = 0; i < 10 && mean > 20.0; ++i) {
        pull(cam, buf);
        mean = meanOf(buf, buf.size());
    }
    check(mean < 20.0, "frames follow it (black)");

    cam.setCameraAttribute("speed", "4");          // type "any" matches a USB camera
    check(cam.error() == ERROR_CODE::NONE && cam.sourceInt("horizontal-speed") == 4,
          "an entry typed \"any\" applies to every camera");
    cam.setCameraAttribute("no such alias", "1");
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE, "unknown alias -> INVALID_ATTRIBUTE");
    cam.setCameraAttribute("csi only", "true");
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE, "an entry typed CSI does not apply to USB");
    cam.setCameraAttribute("pattern", "3x");
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE && cam.sourceInt("pattern") == 2,
          "unparsable value -> INVALID_ATTRIBUTE, property unchanged");
    cam.setCameraAttribute("pattern", "999");
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE && cam.sourceInt("pattern") == 2,
          "value outside the enum -> INVALID_ATTRIBUTE, property unchanged");
    cam.setCameraAttribute("typo", "1");
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE,
          "an entry naming a property the element lacks -> INVALID_ATTRIBUTE (was NONE)");
    cam.setCameraAttribute("speed as text", "7");
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE && cam.sourceInt("horizontal-speed") == 4,
          "a string entry for an int property -> INVALID_ATTRIBUTE (was a wrong-type write)");
    cam.setCameraAttribute("sharpness", "3");
    check(cam.error() == ERROR_CODE::INVALID_ATTRIBUTE,
          "a V4L2 control on a source without extra-controls -> INVALID_ATTRIBUTE");
    cam.close();

    // Rejected while queued: start() still runs, and says which one failed.
    LogSink log;
    TestCamera cam2;
    cam2.setLogCallback(log.callback());
    cam2.setAttributeDictionary(testDictionary());
    cam2.setPipelineParams(fastParams());
    cam2.open();
    cam2.setCameraVideoFormat(0);
    cam2.setCameraAttribute("pattern", "999");
    cam2.setCameraAttribute("speed", "2");
    cam2.start();
    check(cam2.state() == CAMERA_STATUS::RUNNING && cam2.sourceInt("horizontal-speed") == 2,
          "a rejected queued attribute does not stop start() or the others");
    check(log.has("W attribute pattern=999 not applied on videotestsrc"), "... and is logged by name");
    cam2.close();
}

static void testExtraControls() {
    std::printf("\n--- V4L2 controls through v4l2src extra-controls ---\n");
    gst_init(nullptr, nullptr);
    GstElement* v4l2 = gst_element_factory_make("v4l2src", nullptr);
    if (!v4l2) {
        std::printf("  skip  no v4l2src (gst-plugins-good) here\n");
        return;
    }
    gst_object_ref_sink(v4l2);
    GstStructure* controls = nullptr;
    const bool a = TestCamera::setExtraControl(v4l2, controls, "gain", 10);
    const bool b = TestCamera::setExtraControl(v4l2, controls, "white_balance_temperature", 4600);
    const bool c = TestCamera::setExtraControl(v4l2, controls, "gain", 12);   // replaces, keeps the other
    GstStructure* set = nullptr;
    g_object_get(v4l2, "extra-controls", &set, nullptr);
    gint gain = -1, wb = -1;
    const bool readBack = set && gst_structure_get_int(set, "gain", &gain) &&
                          gst_structure_get_int(set, "white_balance_temperature", &wb);
    check(a && b && c && readBack && gain == 12 && wb == 4600 && gst_structure_n_fields(set) == 2,
          "controls accumulate in one structure on the element (gain=12, wb=4600)");
    if (set) gst_structure_free(set);
    if (controls) gst_structure_free(controls);
    gst_object_unref(v4l2);

    GstElement* test = gst_element_factory_make("videotestsrc", nullptr);
    gst_object_ref_sink(test);
    GstStructure* none = nullptr;
    check(!TestCamera::setExtraControl(test, none, "gain", 1) && none == nullptr,
          "an element without extra-controls is refused, nothing allocated");
    gst_object_unref(test);
}

/// config/camera_attributes.xml from the repo root, or the build's seeded copy.
static bool loadRepoDictionary(AttributeDictionary& dict) {
    if (AttributeDictionary::load("config/camera_attributes.xml", dict)) return true;
    char exe[4096];
    const ssize_t n = ::readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n <= 0) return false;
    std::string dir(exe, static_cast<size_t>(n));
    dir = dir.substr(0, dir.find_last_of('/'));
    return AttributeDictionary::load(dir + "/config/camera_attributes.xml", dict);
}

/// Would @p type's parsed value convert to @p propType?
static bool convertible(AttributeValueType type, GType propType) {
    switch (type) {
        case AttributeValueType::Int:          return g_value_type_transformable(G_TYPE_INT, propType);
        case AttributeValueType::Float:        return g_value_type_transformable(G_TYPE_FLOAT, propType);
        case AttributeValueType::Bool:
        case AttributeValueType::BoolFromZero: return g_value_type_transformable(G_TYPE_BOOLEAN, propType);
        case AttributeValueType::String:
        case AttributeValueType::RangeString:  return g_value_type_transformable(G_TYPE_STRING, propType);
        case AttributeValueType::V4l2Control:  return false;
    }
    return false;
}

static void testRepoDictionary() {
    std::printf("\n--- config/camera_attributes.xml against the real elements ---\n");
    gst_init(nullptr, nullptr);
    AttributeDictionary dict;
    check(loadRepoDictionary(dict) && !dict.entries.empty(), "the repository dictionary loads");
    struct Source { const char* type; const char* factory; };
    for (const Source src : {Source{"USB", "v4l2src"}, Source{"CSI", "nvarguscamerasrc"}}) {
        GstElement* el = gst_element_factory_make(src.factory, nullptr);
        if (!el) {
            std::printf("  skip  %s entries: no %s here\n", src.type, src.factory);
            continue;
        }
        gst_object_ref_sink(el);
        int checked = 0;
        std::string bad;
        for (const auto& e : dict.entries) {
            if (e.type != src.type && e.type != "any") continue;
            ++checked;
            if (e.valueType == AttributeValueType::V4l2Control) {
                GParamSpec* ps = g_object_class_find_property(G_OBJECT_GET_CLASS(el), "extra-controls");
                if (!ps || ps->value_type != GST_TYPE_STRUCTURE) bad += " " + e.gstProperty + "(no extra-controls)";
                continue;
            }
            GParamSpec* ps = g_object_class_find_property(G_OBJECT_GET_CLASS(el), e.gstProperty.c_str());
            if (!ps)                                       bad += " " + e.gstProperty + "(missing)";
            else if (!(ps->flags & G_PARAM_WRITABLE))      bad += " " + e.gstProperty + "(read-only)";
            else if (!convertible(e.valueType, ps->value_type))
                bad += " " + e.gstProperty + "(type " + g_type_name(ps->value_type) + ")";
        }
        gst_object_unref(el);
        check(checked > 0 && bad.empty(), std::string(src.type) + ": all " + std::to_string(checked) +
              " entries name a writable " + src.factory + " property of a fitting type, or a V4L2 control" +
              (bad.empty() ? "" : " — bad:" + bad));
    }
}

static void testStartFailures() {
    std::printf("\n--- start failures (logged with the reason) ---\n");
    LogSink log;
    TestCamera cam;
    cam.setLogCallback(log.callback());
    cam.setPipelineParams(fastParams());
    cam.open();
    cam.setCameraVideoFormat(0);

    cam.description = "nosuchelement name=camerasrc ! fakesink";
    cam.start();
    check(cam.state() == CAMERA_STATUS::ERROR && cam.error() == ERROR_CODE::USB_PIPELINE_ERROR,
          "unknown element -> ERROR, pipelineError()");
    check(log.has("pipeline description rejected") && log.has("nosuchelement"),
          "the parse error names the missing element");

    cam.close();
    cam.open();
    cam.setCameraVideoFormat(0);
    log.clear();
    cam.description = "videotestsrc name=camerasrc ! tee name=srctee srctee. ! fakesink";
    cam.start();
    check(cam.state() == CAMERA_STATUS::ERROR && log.has("required pipeline elements not found"),
          "a pipeline without capvalve/mysink is refused");

    cam.close();
    cam.open();
    cam.setCameraVideoFormat(0);
    log.clear();
    cam.description = "filesrc name=camerasrc location=/nonexistent/camera"
                      " ! video/x-raw" + std::string(" ! tee name=srctee srctee. ! queue ! valve name=capvalve"
                      " ! appsink name=mysink");
    cam.start();
    check(cam.state() == CAMERA_STATUS::ERROR, "a source that cannot open -> ERROR");
    check(log.has("pipeline refused to start") && log.has("camerasrc:") && log.has("/nonexistent/camera"),
          "the log names the element and its reason");

    // A pipeline that never reaches PLAYING, shaped like the old failure: the
    // appsink prerolls and blocks, a second branch drops everything (no GAPs)
    // so its sink never prerolls.  start() must give up at its timeout and
    // tear down without sending EOS, which deadlocked on exactly this shape.
    cam.close();
    PipelineParams quick = fastParams();
    quick.stateChangeTimeoutMs = 800;
    cam.setPipelineParams(quick);
    cam.open();
    cam.setCameraVideoFormat(0);
    log.clear();
    cam.description = "videotestsrc name=camerasrc is-live=true ! tee name=srctee"
                      " srctee. ! queue ! valve name=capvalve drop-mode=1 ! appsink name=mysink"
                      " srctee. ! queue ! valve drop=true drop-mode=1 ! fakesink";
    const auto t0 = std::chrono::steady_clock::now();
    cam.start();
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    check(cam.state() == CAMERA_STATUS::ERROR && log.has("did not reach PLAYING within 800 ms"),
          "a pipeline stuck short of PLAYING fails at the timeout, and says so");
    check(sec < 3.0, "... and its teardown does not hang (" + std::to_string(sec).substr(0, 4) + " s)");

    cam.close();
    cam.setPipelineParams(fastParams());
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.description.clear();
    cam.start();
    check(cam.state() == CAMERA_STATUS::RUNNING, "close() + open() recovers from ERROR");
    cam.close();
}

static void testRuntimeBusMessages() {
    std::printf("\n--- runtime bus messages ---\n");
    LogSink log;
    TestCamera cam;
    cam.setLogCallback(log.callback());
    cam.setPipelineParams(fastParams());
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();

    for (int i = 0; i < 12; ++i) cam.warnSource(("warning " + std::to_string(i)).c_str());
    check(cam.state() == CAMERA_STATUS::RUNNING, "warnings do not stop the camera");
    check(log.has("W pipeline warning on videotestsrc: camerasrc: warning 0"), "a warning is logged with its element");
    check(log.has("warning 9 (further warnings not logged)") && !log.has("warning 10"),
          "warnings are capped at 10 per run");

    cam.failSource("v4l2 dequeue: No such device");
    check(cam.state() == CAMERA_STATUS::ERROR && cam.error() == ERROR_CODE::USB_PIPELINE_ERROR,
          "a runtime error turns RUNNING into ERROR on the next status poll");
    check(log.has("pipeline bus error on videotestsrc: camerasrc: simulated device failure"
                  " (v4l2 dequeue: No such device)"),
          "the error is logged with its element and debug detail");
    std::vector<uint8_t> buf(size_t(kW) * kH * 3);
    uint32_t written = 1;
    cam.captureFrame(buf.data(), static_cast<uint32_t>(buf.size()), written);
    check(written == 0, "no frames in ERROR");
    cam.close();
    check(cam.state() == CAMERA_STATUS::CLOSED, "close() from ERROR -> CLOSED");
}

static void testConcurrentCaptureAndStop() {
    std::printf("\n--- capture racing stop() ---\n");
    TestCamera cam;
    cam.setPipelineParams(fastParams());
    cam.open();
    cam.setCameraVideoFormat(0);
    for (int round = 0; round < 3; ++round) {
        cam.start();
        std::atomic<bool> run{true};
        std::atomic<int>  got{0};
        std::thread t([&] {
            std::vector<uint8_t> buf(size_t(kW) * kH * 3);
            while (run) {
                uint32_t w = 0;
                cam.captureFrame(buf.data(), static_cast<uint32_t>(buf.size()), w);
                if (w) ++got;
            }
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        cam.stop();
        run = false;
        t.join();
        check(got > 0 && cam.state() == CAMERA_STATUS::OPEN,
              "round " + std::to_string(round + 1) + ": frames while running, stop() under a puller");
    }
    cam.close();
}

static void testPipelineStrings() {
    std::printf("\n--- USB / CSI pipeline strings ---\n");
    cameraInfo usb;
    usb.type    = CAMERA_TYPE::USB;
    usb.address = "/dev/video0";
    const std::string mj = UsbProbe(usb).pipeline({1920, 1080, 30.0f, V4L2_PIX_FMT_MJPEG, "MJPG"});
    check(mj == "v4l2src name=camerasrc device=/dev/video0 ! image/jpeg, width=1920, height=1080,"
                " framerate=30/1 ! jpegdec ! video/x-raw ! tee name=srctee srctee. ! queue"
                " max-size-buffers=2 leaky=2 ! valve name=capvalve drop-mode=1 ! videoconvert !"
                " video/x-raw, format=(string)BGR ! appsink name=mysink drop=true max-buffers=1"
                " emit-signals=false sync=false",
          "USB MJPEG 1080p30: the exact string");
    const std::string yu = UsbProbe(usb).pipeline({640, 480, 29.97f, V4L2_PIX_FMT_YUYV, "YUYV"});
    check(yu.find("video/x-raw, format=(string)YUY2, width=640, height=480, framerate=2997/100") !=
              std::string::npos,
          "USB YUYV: raw caps with the reduced NTSC rate");

    cameraInfo csi;
    csi.type     = CAMERA_TYPE::CSI;
    csi.address  = "/dev/video2";
    csi.deviceId = 1;
    CsiProbe cp(csi);
    cp.setOutputResolution(1280, 720, 20.0f);
    const std::string c = cp.pipeline({1920, 1080, 60.0f, V4L2_PIX_FMT_NV12, "NV12"});
    check(c == "nvarguscamerasrc name=camerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=1920,"
               " height=1080, format=(string)NV12, framerate=60/1 ! nvvidconv !"
               " video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12 ! videorate !"
               " video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12,"
               " framerate=20/1 ! tee name=srctee srctee. ! queue max-size-buffers=2 leaky=2 !"
               " valve name=capvalve drop-mode=1 ! nvvidconv ! video/x-raw, format=(string)BGRx !"
               " videoconvert ! video/x-raw, format=(string)BGR ! appsink name=mysink drop=true"
               " max-buffers=1 emit-signals=false sync=false",
          "CSI 1080p60 scaled to 720p20: the exact string");
}

int main() {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);   // progress visible even if a test hangs
    std::printf("camera_gst_test — Camera_GST over videotestsrc (no camera needed)\n");
    testLifecycleAndCapture();
    testCaptureValve();
    testBranches();
    testAttributes();
    testExtraControls();
    testRepoDictionary();
    testStartFailures();
    testRuntimeBusMessages();
    testConcurrentCaptureAndStop();
    testPipelineStrings();
    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails,
                g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
