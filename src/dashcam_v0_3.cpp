// dashcam_v0_3.cpp — dashcam software v0.3
//
// v0.3 = v0.2 (UVC passthrough recording + IMX296 lane detection) plus driver
// drowsiness monitoring (libdriverstate: YuNet DNN face crop + binary
// ResNet18 TensorRT classifier) on a driver-facing UVC camera at 2 Hz.
//
// Camera roles:
//   USB UVC (road-facing)  -> PRIMARY footage.  The camera's own compressed
//       MJPEG/H264 stream is muxed straight into MKV (librecord passthrough);
//       telemetry goes to an .ass sidecar.
//   IMX296 CSI (Argus)     -> lane detection inference ONLY (liblanedetector).
//   USB UVC (driver-facing)-> drowsiness classification ONLY (libdriverstate):
//       raw YUYV branch rate-limited to 2 fps, face crop, DROWSY/NATURAL.
//
// Startup configuration setter (resolveCameraConfiguration, in the init phase):
// inspects which cameras are present, assigns each discovered camera a role,
// then logs the recognised profile.  The two anchor cases:
//   * one USB camera and nothing else   -> RECORD-ONLY   (dashcam footage only).
//   * one USB camera + one IMX296 CSI   -> RECORD + LANES (footage on the USB,
//                                          lane detection on the IMX296).
// Driver monitoring layers a third role on top when a cabin camera is available
// (see below).  Any role that cannot be filled simply stays OFF.
//
// Driver-camera selection (librecord opens its camera exclusively, so the
// recorder and the driver monitor can never share one UVC device):
//   - Pinned: a <Camera name="cabin" type="USB"> config entry with a non-empty
//     <Device> (e.g. /dev/video2) reserves that device for driver monitoring;
//     recording takes the first OTHER precompressed-capable USB camera.  If
//     that leaves no recordable camera the app logs an ERROR and runs
//     WITHOUT recording (the operator explicitly assigned the camera away).
//     <Enabled>false</Enabled> on the cabin entry disables driver monitoring.
//   - Auto (no cabin entry / empty Device): first USB camera records (v0.2
//     behaviour) and the first REMAINING USB camera with a raw YUYV format
//     becomes the driver camera; with a single USB camera plugged in, driver
//     monitoring stays off.
//
// Every subsystem degrades gracefully: a missing engine, face model, or camera
// logs an ERROR and the rest keeps running.
//
// Driver alerts, two independent layers (tuned via <DriverScore>):
//   ACUTE  — sustained instantaneous DROWSY (AcuteAlertSec) -> immediate WARN
//            micro-sleep alert; wire a GPIO/MIDI alarm there later.
//   SCORE  — long-horizon fatigue score (libdriverstate FatigueScorer):
//            100 down past 0; -10 per completed 10 s drowsy, +5 per 10 s
//            awake, -10 per drowsiness-correlated lane drift (lateral offset
//            bridged from liblanedetector), cap decays 10/driving-hour.
//            Graded OK/CAUTION/WARNING tiers; FATIGUE (score <= 0 sustained
//            5 min) logs at ERROR — the "pull over" alarm.  SIGUSR1 resets
//            the score to a fresh session (GPIO button in a later version).
//
// Build: `make` (target dashcam_v0_3; needs TRT + OpenCV, no VPI dependency).
// Run:   docker_dev/launchcode_v0_3.sh   (starts this binary immediately)

#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libcommlink.h"
#include "libconfig.h"
#include "libdriverstate.h"
#include "liblanedetector.h"
#include "liblog.h"
#include "libnetwork.h"
#include "librecord.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gst/gst.h>
#include <linux/videodev2.h>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>   // inet_pton — RTP-host validation for remote-control commands
#include <netinet/in.h>

using namespace dashcam::camera;
namespace fs = std::filesystem;

// ─── shutdown / reset flags ───────────────────────────────────────────────────

static volatile std::sig_atomic_t g_run = 1;
static void onSignal(int) { g_run = 0; }

// SIGUSR1 = software fatigue-score reset ("I took a break").  A GPIO button
// will trigger the same path in a later version.
static volatile std::sig_atomic_t g_resetScore = 0;
static void onResetScore(int) { g_resetScore = 1; }

// ─── driver-alert tuning ──────────────────────────────────────────────────────

// The ACUTE micro-sleep alert (sustained instantaneous DROWSY) is separate
// from — and never replaced by — the long-horizon fatigue score.  Its fire
// window comes from <DriverScore> AcuteAlertSec; the clear window stays fixed.
static constexpr float kAlertClearSec  = 2.0f;
// Sustained no-face before the "driver not visible" notice (the detector
// already bridges short dropouts with its faceHoldSec grace window).
static constexpr float kNoFaceSec      = 5.0f;
// While the high-confidence FATIGUE level is latched, re-raise the "pull over"
// alarm every this-many seconds — a one-shot alert a fatigued driver misses is
// no alert.  (A GPIO/audio alarm would repeat on the same cadence.)
static constexpr float kFatigueRealertSec = 30.0f;
static constexpr auto  kInferenceStartupTimeout = std::chrono::seconds(8);
static constexpr uint32_t kLaneFreshnessMs = 3000;
static constexpr uint32_t kDriverFreshnessMs = 5000;
/// Vehicle telemetry arrives at ~10 Hz; past this the overlay reverts to
/// defaults rather than showing a position the car has since driven away from.
static constexpr int      kBridgeFreshnessMs = 1000;

// ─── helpers ─────────────────────────────────────────────────────────────────

static std::string localTimestamp() {
    auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm tm{};
    localtime_r(&tt, &tm);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

static int64_t epochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static dashcam::camera::PipelineParams
makePipelineParams(const dashcam::config::PipelineConfig& p) {
    dashcam::camera::PipelineParams pp;
    pp.captureTimeoutMs     = static_cast<uint32_t>(p.captureTimeoutMs);
    pp.stateChangeTimeoutMs = static_cast<uint32_t>(p.stateChangeTimeoutMs);
    pp.eosTimeoutMs         = static_cast<uint32_t>(p.eosTimeoutMs);
    pp.captureQueueDepth    = static_cast<uint32_t>(p.captureQueueDepth);
    pp.appsinkMaxBuffers    = static_cast<uint32_t>(p.appsinkMaxBuffers);
    pp.branchQueueDepth     = static_cast<uint32_t>(p.branchQueueDepth);
    return pp;
}

static dashcam::log::LogParams
makeLogParams(const dashcam::config::LogConfig& l) {
    dashcam::log::LogParams lp;
    lp.queueSize     = static_cast<uint32_t>(l.queueSize);
    lp.rotateSizeKb  = static_cast<uint32_t>(l.rotateSizeKb);
    lp.rotateFiles   = static_cast<uint32_t>(l.rotateFiles);
    lp.flushEverySec = static_cast<uint32_t>(l.flushEverySec);
    lp.level         = l.level;
    lp.flushOn       = l.flushOn;
    return lp;
}

// Query internet time (SNTP) for clock-health telemetry.  Plain SNTP has no
// cryptographic server authentication, so it must never be used to set the
// privileged system clock; clock discipline belongs to the host time service.
// Called before log::init(), with a summary repeated into the persistent log.
static std::string syncSystemTime(const dashcam::config::NetworkConfig& net,
                                  const dashcam::log::LogCallback& log) {
    using LvL = dashcam::log::LogLevel;
    namespace nw = dashcam::network;

    if (!net.timeSyncEnabled) {
        log(LvL::INFO, "time sync: disabled (<Network><TimeSyncEnabled> = false)");
        return "time sync: disabled";
    }

    const std::string server = net.ntpServer;
    const int tries = 1 + std::max(0, (int)net.ntpRetries);
    nw::TimeResult t;
    for (int i = 0; i < tries && !t.valid; ++i) {
        if (i > 0)
            log(LvL::INFO, "time sync: retry " + std::to_string(i) + "/" +
                           std::to_string(tries - 1) + " to " + server);
        t = nw::queryTime(server, static_cast<uint16_t>((int)net.ntpPort),
                          (int)net.ntpTimeoutMs, log);
    }

    if (!t.valid) {
        log(LvL::WARN, "time sync: no NTP reply from " + server + " after " +
                       std::to_string(tries) + " attempt(s); keeping current clock");
        return "time sync: FAILED (" + server + ", " + std::to_string(tries) + " tries)";
    }

    char off[48];
    std::snprintf(off, sizeof(off), "%+.3f", t.offsetSeconds);
    const std::string base = "time sync: " + server + " offset " + off + " s";

    log(LvL::INFO, base + " (observation only; host time service owns clock discipline)");
    return base + " (observation only)";
}

// Build an H.264-over-RTP output branch bin from an RtpSession description, ready
// for Camera_GST::addBranch().  gst_parse_bin_from_description ghosts the head
// element's sink pad so the tee's queue/valve can link to it.  Returns nullptr on
// a parse failure (the caller then simply runs without the RTP stream).
static GstElement* makeRtpBranchBin(const dashcam::network::RtpSession& rtp, bool nvmm,
                                    float fps, const std::string& sinkName,
                                    const dashcam::log::LogCallback& log) {
    GError* err = nullptr;
    const std::string desc = rtp.branchDescription(nvmm, fps, sinkName);
    GstElement* bin = gst_parse_bin_from_description(desc.c_str(), TRUE, &err);
    if (!bin || err) {
        log(dashcam::log::LogLevel::ERROR,
            std::string("RTP branch parse failed (streaming off): ") + (err ? err->message : "?"));
        if (err) g_error_free(err);
        if (bin) gst_object_unref(bin);
        return nullptr;
    }
    return bin;
}

// Escape a string for embedding inside a JSON double-quoted value.  The RTP host
// can be operator-supplied ("RTP lane <host>"); even though isValidRtpHost()
// already blocks quotes, escaping here is defence-in-depth so a malformed host can
// never corrupt or inject telemetry fields.
static std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (unsigned char ch : s) {
        switch (ch) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (ch < 0x20) {
                    char b[8];
                    std::snprintf(b, sizeof(b), "\\u%04x", ch);
                    out += b;
                } else {
                    out.push_back(static_cast<char>(ch));
                }
        }
    }
    return out;
}

// Validate an RTP destination host before it reaches udpsink / telemetry: accept
// an IPv4 literal or an RFC-1123 hostname (letters, digits, '-', dot-separated
// labels that neither start nor end with '-'), reject everything else.  Blocks a
// remote operator from wedging a malformed host into GStreamer or the JSON.
static bool isValidRtpHost(const std::string& h) {
    if (h.empty() || h.size() > 253) return false;
    struct in_addr a;
    if (::inet_pton(AF_INET, h.c_str(), &a) == 1) return true;   // IPv4 literal

    size_t labelLen = 0;
    for (size_t i = 0; i < h.size(); ++i) {
        const char ch = h[i];
        if (ch == '.') {
            if (labelLen == 0) return false;           // empty label ("a..b" / leading dot)
            if (h[i - 1] == '-') return false;         // label ended with '-'
            labelLen = 0;
            continue;
        }
        const bool alnumDash =
            (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
            (ch >= '0' && ch <= '9') || ch == '-';
        if (!alnumDash) return false;
        if (labelLen == 0 && ch == '-') return false;  // label started with '-'
        if (++labelLen > 63) return false;
    }
    return labelLen != 0 && h.back() != '-';           // no trailing dot; last label ok
}

// Compact single-line JSON telemetry record pushed to remote clients over the
// ControlServer channel (see the main loop).  Carries the ADAS state the device
// computes onboard plus the current RTP destinations; a remote viewer parses
// this alongside the H.264/RTP video to render a full remote-monitoring view.
//
// Each source reports BOTH "on" (detector configured) and "valid" (a fresh sample
// backs the numbers this tick), so a consumer distinguishes OFF (on=false) from
// WARMING (on=true, valid=false) from a real reading (valid=true) — the numbers of
// an invalid source are sentinels, not stale/fabricated state.
static std::string makeTelemetryJson(const dashcam::record::OverlayData& od,
                                     bool laneOn, bool drvOn, bool acuteAlert,
                                     const std::string& laneHost, int lanePort,
                                     const std::string& drvHost, int drvPort) {
    auto jb = [](bool b) { return b ? "true" : "false"; };
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << "{\"t\":" << od.timestampMs
       << ",\"lane\":{\"on\":"  << jb(laneOn)
       << ",\"valid\":"         << jb(od.laneValid)
       << ",\"n\":"             << od.laneCount
       << ",\"ego\":"           << od.egoLaneIndex;
    os.precision(3);
    os << ",\"off\":"      << od.laneOffset
       << ",\"offValid\":" << jb(od.laneOffsetValid) << "}"
       << ",\"driver\":{\"on\":" << jb(drvOn)
       << ",\"valid\":"          << jb(od.driverValid);
    os.precision(1);
    os << ",\"fatigue\":" << od.fatigueScore
       << ",\"level\":"   << od.fatigueLevel
       << ",\"drowsy\":"  << jb(od.driverDrowsy)
       << ",\"face\":"    << jb(od.faceDetected)
       << ",\"alert\":"   << jb(acuteAlert) << "}"
       << ",\"rtp\":{\"lane\":\""    << jsonEscape(laneHost) << ":" << lanePort << "\""
       <<           ",\"driver\":\"" << jsonEscape(drvHost)  << ":" << drvPort  << "\"}}";
    return os.str();
}

static dashcam::lane::LaneDetectorConfig
makeLaneConfig(const dashcam::config::DetectionConfig& d,
               const dashcam::log::LogCallback& log) {
    dashcam::lane::LaneDetectorConfig c;
    c.enginePath      = (std::string)d.laneEnginePath;
    c.targetHz        = static_cast<uint32_t>((int)d.laneTargetHz);
    c.branchMaxFps    = static_cast<uint32_t>((int)d.laneBranchMaxFps);
    c.inputCropTop    = (float)d.laneInputCropTop;
    c.inputCropBottom = (float)d.laneInputCropBottom;
    c.laneReferenceY  = (float)d.laneReferenceY;
    c.sourceIsNVMM    = true;                   // IMX296 tee emits NVMM
    c.log             = log;
    return c;
}

static dashcam::driver::FatigueScoreConfig
makeScoreConfig(const dashcam::config::DriverScoreConfig& s) {
    dashcam::driver::FatigueScoreConfig c;
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

static dashcam::driver::DriverStateConfig
makeDriverConfig(const dashcam::config::DetectionConfig& d,
                 const dashcam::config::DriverScoreConfig& s,
                 const dashcam::log::LogCallback& log) {
    dashcam::driver::DriverStateConfig c;
    c.enginePath      = (std::string)d.driverEnginePath;
    c.targetHz        = static_cast<uint32_t>((int)d.driverTargetHz);
    c.branchMaxFps    = static_cast<uint32_t>((int)d.driverBranchMaxFps);
    c.drowsyThreshold = (float)d.driverDrowsyThreshold;
    c.faceDetection      = (bool)d.driverFaceDetection;
    c.faceModelPath      = (std::string)d.driverFaceModelPath;
    c.faceScoreThreshold = (float)d.driverFaceScore;
    c.faceDetectScale    = (float)d.driverFaceDetectScale;
    c.sourceIsNVMM       = false;               // driver camera is USB/UVC
    c.score           = makeScoreConfig(s);
    c.log             = log;
    return c;
}

static const char* levelStr(dashcam::driver::FatigueLevel l) {
    using dashcam::driver::FatigueLevel;
    switch (l) {
        case FatigueLevel::OK:      return "OK";
        case FatigueLevel::CAUTION: return "CAUTION";
        case FatigueLevel::WARNING: return "WARNING";
        case FatigueLevel::FATIGUE: return "FATIGUE";
    }
    return "?";
}

// True if this video node's sensor sysfs card name contains `needle`.
static bool sensorNameContains(const cameraInfo& info, const char* needle) {
    const std::string::size_type slash = info.address.find_last_of('/');
    const std::string node = (slash == std::string::npos)
                           ? info.address : info.address.substr(slash + 1);
    std::ifstream f("/sys/class/video4linux/" + node + "/name");
    if (!f) return false;
    std::string name((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    return name.find(needle) != std::string::npos;
}

using SensorNameMatcher = bool (*)(const cameraInfo&, const char*);

// Recording format: PRECOMPRESSED only (MJPEG/H264 — librecord passthrough).
// Largest area <=1080p at ~24-31 fps; H264 preferred over MJPEG when both
// exist (smaller files at the same zero encode cost).
static int pickUsbRecordFormat(const cameraInfo& ci) {
    long bestH264Area = -1, bestMjpgArea = -1;
    int bestH264 = -1, bestMjpg = -1;
    for (size_t i = 0; i < ci.videoFormats.size(); ++i) {
        const auto& f = ci.videoFormats[i];
        if (f.frameRate < 24.0f || f.frameRate > 31.0f) continue;
        if (f.width > 1920 || f.height > 1080) continue;
        const long area = static_cast<long>(f.width) * f.height;
        if (f.pixelFormat == V4L2_PIX_FMT_H264) {
            if (area > bestH264Area) { bestH264Area = area; bestH264 = (int)i; }
        } else if (f.pixelFormat == V4L2_PIX_FMT_MJPEG) {
            if (area > bestMjpgArea) { bestMjpgArea = area; bestMjpg = (int)i; }
        }
    }
    return bestH264 >= 0 ? bestH264 : bestMjpg;
}

// Driver-monitor format: raw YUYV so the branch can convert to BGR without a
// JPEG decode.  Prefer 640x480 (square-crops well for the 224x224 model); any
// other YUYV mode >=240 rows works as fallback, smallest area first.  Lowest
// native frame rate wins within a size — the branch drops to 2 fps anyway, so
// a slow mode just saves USB bandwidth (the C270 floor is 5 fps).
static int pickDriverFormat(const cameraInfo& ci) {
    int best = -1;
    long bestArea = -1;
    float bestRate = 1e9f;
    for (size_t i = 0; i < ci.videoFormats.size(); ++i) {
        const auto& f = ci.videoFormats[i];
        if (f.pixelFormat != V4L2_PIX_FMT_YUYV) continue;
        if (f.height < 240 || f.frameRate < 2.0f) continue;
        const bool ideal     = (f.width == 640 && f.height == 480);
        const bool bestIdeal = (best >= 0)
            && ci.videoFormats[(size_t)best].width  == 640
            && ci.videoFormats[(size_t)best].height == 480;
        const long area = static_cast<long>(f.width) * f.height;
        const bool better =
            (ideal && !bestIdeal) ||
            (ideal == bestIdeal &&
             (best < 0 || area < bestArea ||
              (area == bestArea && f.frameRate < bestRate)));
        if (better) { best = (int)i; bestArea = area; bestRate = f.frameRate; }
    }
    return best;
}

// ─── camera configuration setter ───────────────────────────────────────────────
// The initialisation-phase step that inspects which cameras are present and
// assigns each a functionality (role).  Two operator-facing scenarios anchor it
// (file header): a lone USB camera records; a USB + IMX296 pair records on the
// USB and runs lane detection on the IMX296.  The v0.3 driver-monitoring camera
// (pinned <Camera name="cabin"> or an auto-picked spare USB) layers on top.
// Every role degrades independently — an unfilled role just stays OFF.

// Roles assigned to the discovered cameras.  Each pointer aliases an element of
// the caller's `cams` vector (nullptr = role unfilled) with its chosen format
// index into that camera's videoFormats.
struct CameraConfiguration {
    const cameraInfo* record    = nullptr;  ///< UVC MJPEG/H264 passthrough recording.
    int               recordFmt = -1;       ///< Format index for `record`.
    const cameraInfo* lane      = nullptr;  ///< IMX296 CSI lane detection (single native mode).
    int               laneFmt   = -1;       ///< Format index for `lane`.
    int               laneSensorId = -1;    ///< Configured Argus sensor id (-1 = discovery).
    const dashcam::config::CameraConfig* laneConfig = nullptr;
    const cameraInfo* driver    = nullptr;  ///< UVC driver-monitoring (drowsiness).
    int               driverFmt = -1;       ///< Format index for `driver`.
    bool              driverPinned = false; ///< Driver camera came from a config pin (vs auto).

    // Discovery census, for the recognised-configuration summary line.
    int usbCount = 0, imx296Count = 0, otherCsiCount = 0;
};

// One-line human label for the recognised configuration (the "profile"), keyed
// on which roles ended up filled.  The first two lines are the operator's named
// scenarios; the rest cover the driver-monitoring and degraded permutations.
static std::string describeConfiguration(const CameraConfiguration& c) {
    const bool r = c.record, l = c.lane, d = c.driver;
    if ( r && !l && !d) return "RECORD-ONLY (single USB dashcam recording)";
    if ( r &&  l && !d) return "RECORD + LANES (USB recording + IMX296 lane detection)";
    if ( r && !l &&  d) return "RECORD + DRIVER-MONITOR (USB recording + drowsiness; no IMX296)";
    if ( r &&  l &&  d) return "RECORD + LANES + DRIVER-MONITOR (full v0.3)";
    if (!r &&  l && !d) return "LANES-ONLY (IMX296 lane detection; no recordable USB)";
    if (!r &&  l &&  d) return "LANES + DRIVER-MONITOR (no recordable USB)";
    if (!r && !l &&  d) return "DRIVER-MONITOR-ONLY (no recordable USB, no IMX296)";
    return "NONE (no camera role could be assigned)";
}

// The configuration setter proper: assign a role to each discovered camera and
// announce the recognised profile.  Assignment order is load-bearing — a pinned
// cabin (driver) camera is reserved BEFORE recording picks, so the recorder
// never grabs the device the operator set aside for driver monitoring.
static CameraConfiguration
resolveCameraConfiguration(const std::vector<cameraInfo>& cams,
                           const std::vector<dashcam::config::CameraConfig>& configs,
                           const dashcam::log::LogCallback& log,
                           SensorNameMatcher sensorMatches = sensorNameContains) {
    using dashcam::log::LogLevel;
    CameraConfiguration cc;
    std::string cabinDev;
    bool cabinEnabled = true;
    for (const auto& cfg : configs) {
        if (cfg.type == "USB" && cfg.name == "cabin") {
            cabinDev = (std::string)cfg.device;
            cabinEnabled = (bool)cfg.enabled;
            break;
        }
    }
    cc.driverPinned = !cabinDev.empty();

    auto exactConfig = [&](const cameraInfo& camera, const char* type)
        -> const dashcam::config::CameraConfig* {
        for (const auto& cfg : configs)
            if (cfg.type == type && !std::string(cfg.device).empty() &&
                (std::string)cfg.device == camera.address)
                return &cfg;
        return nullptr;
    };
    auto disabled = [&](const cameraInfo& camera, const char* type) {
        const auto* cfg = exactConfig(camera, type);
        return cfg && !(bool)cfg->enabled;
    };
    auto findCamera = [&](const std::string& device, CAMERA_TYPE type)
        -> const cameraInfo* {
        for (const auto& camera : cams)
            if (camera.type == type && camera.address == device)
                return &camera;
        return nullptr;
    };
    auto configuredFormat = [&](const dashcam::config::CameraConfig* cfg,
                                const cameraInfo& camera,
                                bool recording) -> int {
        if (!cfg) return recording ? pickUsbRecordFormat(camera)
                                   : pickDriverFormat(camera);
        const int index = (int)cfg->formatIndex;
        if (index >= 0 && static_cast<size_t>(index) < camera.videoFormats.size()) {
            const auto& f = camera.videoFormats[static_cast<size_t>(index)];
            const bool compatible = recording
                ? (f.pixelFormat == V4L2_PIX_FMT_H264 ||
                   f.pixelFormat == V4L2_PIX_FMT_MJPEG)
                : (f.pixelFormat == V4L2_PIX_FMT_YUYV &&
                   f.height >= 240 && f.frameRate >= 2.0f);
            if (compatible) return index;
            log(LogLevel::WARN, camera.address + ": configured FormatIndex " +
                std::to_string(index) + " is incompatible with the " +
                (recording ? "compressed recorder" : "driver monitor") +
                "; selecting a safe format automatically");
        } else {
            log(LogLevel::WARN, camera.address + ": configured FormatIndex " +
                std::to_string(index) + " is out of range; selecting a safe "
                "format automatically");
        }
        return recording ? pickUsbRecordFormat(camera) : pickDriverFormat(camera);
    };

    // Census first.  A disabled exact device entry removes that device from
    // automatic role assignment; unlisted hot-plug cameras remain eligible.
    for (const auto& c : cams) {
        if (c.type == CAMERA_TYPE::USB) { ++cc.usbCount; continue; }
        if (c.type != CAMERA_TYPE::CSI) continue;
        if (sensorMatches(c, "imx296")) {
            ++cc.imx296Count;
        } else {
            ++cc.otherCsiCount;
        }
    }

    // Prefer an enabled, explicitly configured IMX296.  SensorId and
    // FormatIndex are consumed below instead of being silently ignored.
    for (const auto& cfg : configs) {
        if (cfg.type != "CSI" || !(bool)cfg.enabled ||
            std::string(cfg.device).empty())
            continue;
        const cameraInfo* camera =
            findCamera((std::string)cfg.device, CAMERA_TYPE::CSI);
        if (!camera) {
            log(LogLevel::WARN, "configured CSI camera " +
                (std::string)cfg.device + " not found");
            continue;
        }
        if (!sensorMatches(*camera, "imx296")) continue;
        if (camera->videoFormats.empty()) {
            log(LogLevel::ERROR, camera->address +
                ": IMX296 reports no video formats — lane detection OFF for "
                "this device");
            continue;
        }
        cc.lane = camera;
        cc.laneFmt = 0;
        cc.laneConfig = &cfg;
        break;
    }
    if (!cc.lane) {
        for (const auto& camera : cams) {
            if (camera.type != CAMERA_TYPE::CSI ||
                !sensorMatches(camera, "imx296") ||
                disabled(camera, "CSI"))
                continue;
            if (camera.videoFormats.empty()) {
                log(LogLevel::ERROR, camera.address +
                    ": IMX296 reports no video formats — lane detection OFF "
                    "for this device");
                continue;
            }
            cc.lane = &camera;
            cc.laneFmt = 0;
            cc.laneConfig = exactConfig(camera, "CSI");
            break;
        }
    }
    if (cc.laneConfig) {
        cc.laneSensorId = (int)cc.laneConfig->sensorId;
        const int index = (int)cc.laneConfig->formatIndex;
        if (index >= 0 &&
            static_cast<size_t>(index) < cc.lane->videoFormats.size())
            cc.laneFmt = index;
        else
            log(LogLevel::WARN, cc.lane->address + ": configured CSI "
                "FormatIndex is out of range; using 0");
    }

    // Driver camera first (see the ordering note above).
    if (!cabinEnabled) {
        log(LogLevel::INFO,
            "cabin camera disabled in config — driver monitoring OFF");
    } else if (!cabinDev.empty()) {
        for (const auto& c : cams) {
            if (c.type != CAMERA_TYPE::USB || c.address != cabinDev) continue;
            const int idx = configuredFormat(exactConfig(c, "USB"), c, false);
            if (idx < 0)
                log(LogLevel::ERROR,
                    "pinned cabin camera " + cabinDev + " has no usable YUYV "
                    "format — driver monitoring OFF");
            else { cc.driver = &c; cc.driverFmt = idx; }
            break;
        }
        // Reached here inside the pinned branch, so cabinEnabled is true and a
        // null driver means the pin was unusable.  Distinguish "device absent"
        // (logged here) from "device present but no YUYV format" (logged above).
        if (!cc.driver &&
            std::none_of(cams.begin(), cams.end(), [&](const cameraInfo& c) {
                return c.type == CAMERA_TYPE::USB && c.address == cabinDev; }))
            log(LogLevel::ERROR,
                "pinned cabin camera " + cabinDev + " not found — "
                "driver monitoring OFF");
    }

    // Recording camera: explicit non-cabin USB entries get priority.
    for (const auto& cfg : configs) {
        if (cfg.type != "USB" || cfg.name == "cabin" || !(bool)cfg.enabled ||
            std::string(cfg.device).empty())
            continue;
        const cameraInfo* camera =
            findCamera((std::string)cfg.device, CAMERA_TYPE::USB);
        if (!camera ||
            (!cabinDev.empty() && camera->address == cabinDev) ||
            (cc.driver && camera->address == cc.driver->address))
            continue;
        const int idx = configuredFormat(&cfg, *camera, true);
        if (idx >= 0) {
            cc.record = camera;
            cc.recordFmt = idx;
            break;
        }
    }
    // Otherwise auto-select a non-disabled compressed USB camera the driver
    // monitor has not claimed.
    for (const auto& c : cams) {
        if (cc.record) break;
        if (c.type != CAMERA_TYPE::USB) continue;
        if (disabled(c, "USB")) continue;
        // A cabin pin reserves the physical device even when it is absent or
        // cannot supply YUYV.  Role assignment must follow operator intent:
        // never silently turn a driver-facing camera into road footage.
        if (!cabinDev.empty() && c.address == cabinDev) continue;
        if (cc.driver && c.address == cc.driver->address) continue;
        const int idx = configuredFormat(exactConfig(c, "USB"), c, true);
        if (idx >= 0) { cc.record = &c; cc.recordFmt = idx; break; }
    }

    // Auto mode: driver camera = first remaining YUYV-capable USB.
    if (!cc.driver && cabinEnabled && cabinDev.empty()) {
        int eligibleSpareCount = 0;
        for (const auto& c : cams) {
            if (c.type != CAMERA_TYPE::USB) continue;
            if (disabled(c, "USB")) continue;
            if (cc.record && c.address == cc.record->address) continue;
            ++eligibleSpareCount;
            const int idx = configuredFormat(exactConfig(c, "USB"), c, false);
            if (idx >= 0) { cc.driver = &c; cc.driverFmt = idx; break; }
        }
        // A lone recording camera (the RECORD-ONLY scenario) legitimately has no
        // spare for driver monitoring — that is the expected single-camera
        // profile, not an error.  Only flag it when a spare USB existed but was
        // unusable (e.g. a webcam with no raw YUYV mode).
        if (!cc.driver && eligibleSpareCount > 0)
            log(LogLevel::ERROR,
                "free USB camera(s) have no usable YUYV format — driver "
                "monitoring OFF");
    }

    // Per-role absence notices.  Recording is the primary function, so its
    // absence is an ERROR; lane detection is a degradable secondary subsystem
    // and its absence in a USB-only rig is by design (the RECORD-ONLY profile),
    // so it is reported at INFO — the recognised-profile line names it anyway.
    if (!cc.record)
        log(LogLevel::ERROR,
            !cabinDev.empty()
                ? "no USB camera left for recording (cabin pin reserves "
                    + cabinDev + ") — continuing WITHOUT recording"
                : (cc.driver
                    ? "no USB camera left for recording (driver monitor claimed "
                        + cc.driver->address + ") — continuing WITHOUT recording"
                    : "no recordable USB camera found — continuing WITHOUT "
                      "recording"));
    if (!cc.lane)
        log(LogLevel::INFO,
            "no IMX296 CSI camera found — continuing WITHOUT lane detection");

    // Announce the recognised configuration and the assigned roles.
    log(LogLevel::INFO,
        "camera configuration: " + describeConfiguration(cc)
        + "  [discovered " + std::to_string(cams.size()) + ": "
        + std::to_string(cc.usbCount) + " USB, "
        + std::to_string(cc.imx296Count) + " IMX296"
        + (cc.otherCsiCount ? ", " + std::to_string(cc.otherCsiCount) + " other CSI"
                            : std::string())
        + "]");
    if (cc.record) {
        const auto& f = cc.record->videoFormats[(size_t)cc.recordFmt];
        log(LogLevel::INFO,
            "  role recording  -> " + cc.record->address + " "
            + std::to_string(f.width) + "x" + std::to_string(f.height) + "@"
            + std::to_string((int)f.frameRate)
            + (f.pixelFormat == V4L2_PIX_FMT_H264 ? " H264" : " MJPG")
            + " passthrough");
    }
    if (cc.lane) {
        // Selection above establishes this invariant; keep the access
        // non-throwing so a malformed discovery result cannot escape main().
        const auto& f = cc.lane->videoFormats[static_cast<size_t>(cc.laneFmt)];
        log(LogLevel::INFO,
            "  role lanes      -> " + cc.lane->address + " (IMX296) "
            + std::to_string(f.width) + "x" + std::to_string(f.height));
    }
    if (cc.driver) {
        const auto& f = cc.driver->videoFormats[(size_t)cc.driverFmt];
        log(LogLevel::INFO,
            "  role driver     -> " + cc.driver->address + " "
            + std::to_string(f.width) + "x" + std::to_string(f.height) + "@"
            + std::to_string((int)f.frameRate) + " YUYV"
            + (cc.driverPinned ? " (pinned)" : " (auto)"));
    }
    return cc;
}

// Hardware-independent regression coverage for the role resolver's safety
// invariants.  Run with:
//   dashcam_v0_3 --self-test-camera-configuration
// The normal launcher never supplies this flag.
static int runCameraConfigurationSelfTest() {
    using dashcam::log::LogLevel;
    const dashcam::log::LogCallback noLog =
        [](LogLevel, const std::string&) {};

    auto format = [](uint32_t fourcc) {
        cameraVideoFormat f{};
        f.width = 1280;
        f.height = 720;
        f.frameRate = 30.0f;
        f.pixelFormat = fourcc;
        return f;
    };
    auto camera = [](CAMERA_TYPE type, const std::string& address,
                     std::vector<cameraVideoFormat> formats) {
        cameraInfo c{};
        c.type = type;
        c.address = address;
        c.videoFormats = std::move(formats);
        return c;
    };
    auto cabinConfig = [](const std::string& address, bool enabled) {
        dashcam::config::CameraConfig c;
        c.name = "cabin";
        c.type = "USB";
        c.device = address;
        c.enabled = enabled;
        return c;
    };
    auto disabledUsbConfig = [](const std::string& address) {
        dashcam::config::CameraConfig c;
        c.name = "disabled-spare";
        c.type = "USB";
        c.device = address;
        c.enabled = false;
        return c;
    };

    int failures = 0;
    auto check = [&](bool ok, const char* message) {
        std::fprintf(stderr, "camera-config self-test: %s — %s\n",
                     ok ? "PASS" : "FAIL", message);
        if (!ok) ++failures;
    };

    // A pinned MJPEG-only cabin must remain reserved; it cannot become the
    // road-recording camera just because driver monitoring cannot consume it.
    {
        const std::vector<cameraInfo> cams{
            camera(CAMERA_TYPE::USB, "/dev/cabin",
                   {format(V4L2_PIX_FMT_MJPEG)}),
            camera(CAMERA_TYPE::USB, "/dev/road",
                   {format(V4L2_PIX_FMT_MJPEG)})
        };
        const std::vector<dashcam::config::CameraConfig> configs{
            cabinConfig("/dev/cabin", true)
        };
        const auto cc = resolveCameraConfiguration(cams, configs, noLog);
        check(cc.record && cc.record->address == "/dev/road" && !cc.driver,
              "MJPEG-only cabin pin stays reserved from recording");
    }

    // Disabled cameras are not eligible spare driver cameras and therefore
    // must not trigger the "free camera has no YUYV" error.
    {
        std::vector<std::string> errors;
        const dashcam::log::LogCallback capture =
            [&](LogLevel level, const std::string& message) {
                if (level == LogLevel::ERROR) errors.push_back(message);
            };
        const std::vector<cameraInfo> cams{
            camera(CAMERA_TYPE::USB, "/dev/road",
                   {format(V4L2_PIX_FMT_MJPEG)}),
            camera(CAMERA_TYPE::USB, "/dev/disabled",
                   {format(V4L2_PIX_FMT_MJPEG)})
        };
        const std::vector<dashcam::config::CameraConfig> configs{
            disabledUsbConfig("/dev/disabled")
        };
        const auto cc = resolveCameraConfiguration(cams, configs, capture);
        const bool driverError = std::any_of(
            errors.begin(), errors.end(), [](const std::string& message) {
                return message.find("driver monitoring OFF") != std::string::npos;
            });
        check(cc.record && cc.record->address == "/dev/road" && !driverError,
              "disabled USB does not cause a spurious driver-monitor error");
    }

    // Degenerate discovery data must degrade the lane role, never throw from a
    // vector::at() and terminate recording.
    {
        const std::vector<cameraInfo> cams{
            camera(CAMERA_TYPE::CSI, "/dev/imx296-empty", {}),
            camera(CAMERA_TYPE::USB, "/dev/road",
                   {format(V4L2_PIX_FMT_MJPEG)})
        };
        const auto fakeImx296 = [](const cameraInfo& c, const char*) {
            return c.type == CAMERA_TYPE::CSI;
        };
        const auto cc =
            resolveCameraConfiguration(cams, {}, noLog, fakeImx296);
        check(!cc.lane && cc.record && cc.record->address == "/dev/road",
              "empty CSI format list degrades lanes while recording remains active");
    }

    return failures == 0 ? 0 : 1;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc == 2 &&
        std::string(argv[1]) == "--self-test-camera-configuration")
        return runCameraConfigurationSelfTest();

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGUSR1, onResetScore);   // software fatigue-score reset
    gst_init(&argc, &argv);

    // ── config first (so <Log> can shape the logger), stderr until init ──────
    auto log = dashcam::log::getCallback();
    dashcam::config::AppConfig cfg;
    const std::string configsDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultConfigsDir, dashcam::config::kFallbackConfigsName, log);
    if (!dashcam::config::ConfigReader::loadOrCreate(configsDir + "/dashcam.xml", cfg, log))
        log(dashcam::log::LogLevel::WARN, "config load/create failed; using defaults");

    // WiFi bring-up BEFORE anything network-dependent: ask the OS (NetworkManager)
    // to connect to the SSID.  If it cannot be fulfilled we run in OFFLINE MODE —
    // no internet time-sync, no network streaming.  Detail goes to stderr like the
    // other pre-init startup steps; the summary is logged after init().
    bool offline = false;
    std::string wifiSummary;
    {
        dashcam::network::WifiConnectConfig wc;
        wc.enabled         = cfg.network.wifiConnectEnabled;
        wc.ssid            = (std::string)cfg.network.wifiSsid;
        wc.timeoutSec      = (int)cfg.network.wifiTimeoutSec;
        wc.requireInternet = cfg.network.wifiRequireInternet;
        if (wc.enabled) {
            dashcam::network::WifiStatus ws = dashcam::network::connectWifi(wc, log);
            offline = (ws.state == dashcam::network::ConnectivityState::Offline);
            wifiSummary = offline ? ("network: OFFLINE MODE — " + ws.detail)
                                  : ("network: ONLINE (WiFi '" + ws.ssid + "')");
        } else {
            wifiSummary = "network: WiFi auto-connect disabled (assuming network present)";
        }
    }

    // Internet time sync BEFORE log::init(), so a Jetson that booted with a
    // wrong/unset RTC names its log file — and stamps all footage — with the
    // corrected clock (detail goes to stderr; the summary is logged below).
    // Skipped in offline mode (there is no internet to reach).
    const std::string timeSyncSummary =
        offline ? std::string("time sync: skipped (offline mode)")
                : syncSystemTime(cfg.network, log);

    const std::string logDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultLogDir, dashcam::config::kFallbackLogName, log);
    dashcam::log::init(logDir, makeLogParams(cfg.log));
    log(dashcam::log::LogLevel::INFO,
        "dashcam v0.3 starting (UVC record + IMX296 lanes + driver monitoring)");
    log(dashcam::log::LogLevel::INFO, wifiSummary);
    log(dashcam::log::LogLevel::INFO, timeSyncSummary);

    dashcam::camera::AttributeDictionary dict;
    std::string attrPath = configsDir + "/camera_attributes.xml";
    if (!fs::exists(attrPath))
        attrPath = dashcam::config::configDir() + "/camera_attributes.xml";
    if (!dashcam::camera::AttributeDictionary::load(attrPath, dict, log))
        log(dashcam::log::LogLevel::WARN, "attribute dictionary not loaded");

    const std::string footageDir = dashcam::config::resolveStorageDir(
        cfg.system.footagePath, dashcam::config::kFallbackFootageName, log);

    // ── camera discovery: USB record + USB driver + IMX296 lanes ─────────────
    std::vector<cameraInfo> cams;
    if (getCameraList(cams, log) != ERROR_CODE::NONE || cams.empty()) {
        log(dashcam::log::LogLevel::ERROR, "no cameras discovered; aborting");
        dashcam::log::shutdown();
        return 1;
    }

    // ── configuration setter: assign a functionality to each camera ──────────
    // Inspect which cameras are present and map each to a role (recording, lane
    // detection, driver monitoring), then announce the recognised profile.
    const CameraConfiguration camCfg =
        resolveCameraConfiguration(cams, cfg.cameras, log);
    cameraInfo laneSelection;
    const cameraInfo* laneInfo = nullptr;
    if (camCfg.lane && camCfg.laneFmt >= 0 &&
        static_cast<size_t>(camCfg.laneFmt) < camCfg.lane->videoFormats.size()) {
        laneSelection = *camCfg.lane;
        if (camCfg.laneSensorId >= 0)
            laneSelection.deviceId =
                static_cast<uint32_t>(camCfg.laneSensorId);
        laneInfo = &laneSelection;
    } else if (camCfg.lane) {
        // Defensive boundary check: resolver currently guarantees a valid
        // format, but discovery/configuration data must never be able to throw
        // std::out_of_range and terminate unrelated recording/driver roles.
        log(dashcam::log::LogLevel::ERROR,
            camCfg.lane->address + ": invalid lane format selection — "
            "continuing WITHOUT lane detection");
    }
    const int         usbFmtIdx = camCfg.recordFmt;
    const int         laneFmtIdx = camCfg.laneFmt;
    const int         drvFmtIdx = camCfg.driverFmt;
    auto validatedRoleCamera =
        [&](const cameraInfo* camera, int formatIndex,
            const char* role) -> const cameraInfo* {
            if (!camera) return nullptr;
            if (formatIndex >= 0 &&
                static_cast<size_t>(formatIndex) < camera->videoFormats.size())
                return camera;
            log(dashcam::log::LogLevel::ERROR,
                camera->address + ": invalid " + role +
                " format selection — role disabled");
            return nullptr;
        };
    const cameraInfo* usbInfo =
        validatedRoleCamera(camCfg.record, usbFmtIdx, "recording");
    const cameraInfo* drvInfo =
        validatedRoleCamera(camCfg.driver, drvFmtIdx, "driver");

    if (!usbInfo && !laneInfo && !drvInfo) {
        log(dashcam::log::LogLevel::ERROR, "no usable cameras at all; aborting");
        dashcam::log::shutdown();
        return 1;
    }

    const auto* laneConfig = laneInfo ? camCfg.laneConfig : nullptr;
    const cameraVideoFormat nativeLaneFmt = laneInfo
        ? laneInfo->videoFormats[static_cast<size_t>(laneFmtIdx)]
        : cameraVideoFormat{};
    const uint32_t laneWidth =
        laneConfig && (int)laneConfig->outWidth > 0
            ? static_cast<uint32_t>((int)laneConfig->outWidth)
            : nativeLaneFmt.width;
    const uint32_t laneHeight =
        laneConfig && (int)laneConfig->outHeight > 0
            ? static_cast<uint32_t>((int)laneConfig->outHeight)
            : nativeLaneFmt.height;
    const float laneFps =
        laneConfig && (float)laneConfig->outFps > 0.0f
            ? (float)laneConfig->outFps
            : nativeLaneFmt.frameRate;

    // ── lane detector: load the TRT engine before pipeline setup ─────────────
    std::unique_ptr<dashcam::lane::LaneDetector> laneDet;
    if (laneInfo) {
        const auto t0 = std::chrono::steady_clock::now();
        try {
            laneDet = std::make_unique<dashcam::lane::LaneDetector>(
                laneWidth, laneHeight, makeLaneConfig(cfg.detection, log));
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            log(dashcam::log::LogLevel::INFO,
                "lane engine loaded in " + std::to_string(ms) + " ms");
        } catch (const std::exception& e) {
            log(dashcam::log::LogLevel::ERROR,
                std::string(e.what()) + " — continuing WITHOUT lane detection");
        }
    }

    // ── driver-state detector: load the TRT engine + YuNet face model ────────
    std::unique_ptr<dashcam::driver::DriverStateDetector> drvDet;
    if (drvInfo) {
        const auto& dFmt = drvInfo->videoFormats[(size_t)drvFmtIdx];
        const auto t0 = std::chrono::steady_clock::now();
        try {
            drvDet = std::make_unique<dashcam::driver::DriverStateDetector>(
                dFmt.width, dFmt.height,
                makeDriverConfig(cfg.detection, cfg.driverScore, log));
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            log(dashcam::log::LogLevel::INFO,
                "driver engine loaded in " + std::to_string(ms) + " ms");
        } catch (const std::exception& e) {
            log(dashcam::log::LogLevel::ERROR,
                std::string(e.what()) + " — continuing WITHOUT driver monitoring");
        }
    }

    // ── inference-camera RTP streaming sessions (control layer) ──────────────
    // Lane/road camera streams to RtpPort, driver camera to RtpPort+2 (distinct
    // ports so the two H.264 streams never collide on one UDP endpoint).
    dashcam::network::RtpSession laneRtp, drvRtp;
    {
        dashcam::network::RtpStreamConfig rc;
        rc.enabled     = cfg.network.rtpEnabled;
        rc.host        = (std::string)cfg.network.rtpHost;
        rc.bitrateKbps = (int)cfg.network.rtpBitrateKbps;
        rc.port = static_cast<uint16_t>((int)cfg.network.rtpPort);       laneRtp.configure(rc);
        rc.port = static_cast<uint16_t>((int)cfg.network.rtpPort + 2);   drvRtp.configure(rc);
    }

    // Shared control state for the remote-control channel (ControlServer, below).
    // The kept udpsink refs let the command handler re-point host/port live; the
    // *Alive flags gate setBranchEnabled so a mid-run camera teardown (in the
    // health block) can't race a handler into a freed valve.  All of it is
    // guarded by ctrlMtx: the handler runs on a ControlServer client thread.
    GstElement* laneSink = nullptr;   // named udpsink of the lane RTP branch (kept ref).
    GstElement* drvSink  = nullptr;   // named udpsink of the driver RTP branch (kept ref).
    std::mutex  ctrlMtx;
    bool        laneRtpAlive = false, drvRtpAlive = false;   // branch attached & camera live.
    bool        laneRtpOn    = true,  drvRtpOn    = true;     // valve pass state.
    std::string laneDestHost = (std::string)cfg.network.rtpHost;
    std::string drvDestHost  = (std::string)cfg.network.rtpHost;
    int         laneDestPort = (int)cfg.network.rtpPort;
    int         drvDestPort  = (int)cfg.network.rtpPort + 2;

    // ── lane camera ──────────────────────────────────────────────────────────
    Camera_CSI laneCam(laneInfo ? *laneInfo : cameraInfo{});
    if (laneDet) {
        laneCam.setLogCallback(log);
        laneCam.setAttributeDictionary(dict);
        laneCam.setPipelineParams(makePipelineParams(cfg.pipeline));
        laneCam.setOutputResolution(laneWidth, laneHeight, laneFps);
        if (laneConfig)
            for (const auto& [name, value] : laneConfig->capabilities)
                laneCam.setCameraAttribute(name, value);

        GstElement* laneBin = laneDet->createBin();
        if (laneBin) {
            laneCam.addBranch("lanes", laneBin, /*leaky=*/true);
        } else {
            log(dashcam::log::LogLevel::ERROR,
                "lane bin creation failed — continuing WITHOUT lane detection");
            laneDet.reset();
        }
    }
    if (laneDet && cfg.network.rtpEnabled && !offline) {
        // CSI/Argus tee is NVMM → nvvidconv head.  Leaky so encoding never blocks
        // the lane inference feed.
        GstElement* rtpBin = makeRtpBranchBin(laneRtp, /*nvmm=*/true, laneFps,
                                              "lane-rtpsink", log);
        if (rtpBin) {
            // Keep a ref to the named udpsink BEFORE addBranch hands the bin to the
            // pipeline, so the control channel can re-point host/port at runtime.
            laneSink = gst_bin_get_by_name(GST_BIN(rtpBin), "lane-rtpsink");
            laneCam.addBranch("lane-rtp", rtpBin, /*leaky=*/true);
            log(dashcam::log::LogLevel::INFO,
                "lane RTP stream -> " + (std::string)cfg.network.rtpHost + ":" +
                std::to_string((int)cfg.network.rtpPort) + "   view: " + laneRtp.viewerHint());
        }
    }
    if (laneDet) {
        laneCam.open();
        laneCam.setCameraVideoFormat(static_cast<uint16_t>(laneFmtIdx));
        laneCam.start();

        cameraStatus ls;
        laneCam.getCameraStatus(ls);
        if (ls.status != CAMERA_STATUS::RUNNING) {
            log(dashcam::log::LogLevel::ERROR,
                "IMX296 start failed — continuing WITHOUT lane detection");
            laneCam.stop(); laneDet.reset();
            laneCam.close();
        } else {
            laneCam.setCaptureEnabled(false);   // branch-only consumer
            laneDet->start();
            const auto deadline =
                std::chrono::steady_clock::now() + kInferenceStartupTimeout;
            while (g_run && laneDet->processedFrameCount() == 0 &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                laneCam.getCameraStatus(ls);
                if (ls.status != CAMERA_STATUS::RUNNING) break;
            }
            if (laneDet->processedFrameCount() == 0) {
                log(dashcam::log::LogLevel::ERROR,
                    "lane path produced no inference result before timeout — "
                    "continuing WITHOUT lane detection");
                laneCam.stop(); laneDet->stop(); laneDet.reset();
                laneCam.close();
            } else {
                log(dashcam::log::LogLevel::INFO,
                    "lane inference ready (first result received)");
            }
        }
    }

    // ── driver camera ────────────────────────────────────────────────────────
    Camera_USB drvCam(drvInfo ? *drvInfo : cameraInfo{});
    if (drvDet) {
        drvCam.setLogCallback(log);
        drvCam.setPipelineParams(makePipelineParams(cfg.pipeline));

        GstElement* drvBin = drvDet->createBin();
        if (drvBin) {
            drvCam.addBranch("driverstate", drvBin, /*leaky=*/true);
        } else {
            log(dashcam::log::LogLevel::ERROR,
                "driver bin creation failed — continuing WITHOUT driver monitoring");
            drvDet.reset();
        }
    }
    if (drvDet && cfg.network.rtpEnabled && !offline) {
        // USB driver cam tee is system-memory raw → videoconvert head.
        const float drvFps = drvInfo->videoFormats[(size_t)drvFmtIdx].frameRate;
        GstElement* rtpBin = makeRtpBranchBin(drvRtp, /*nvmm=*/false, drvFps,
                                              "driver-rtpsink", log);
        if (rtpBin) {
            drvSink = gst_bin_get_by_name(GST_BIN(rtpBin), "driver-rtpsink");
            drvCam.addBranch("driver-rtp", rtpBin, /*leaky=*/true);
            log(dashcam::log::LogLevel::INFO,
                "driver RTP stream -> " + (std::string)cfg.network.rtpHost + ":" +
                std::to_string((int)cfg.network.rtpPort + 2) + "   view: " + drvRtp.viewerHint());
        }
    }
    if (drvDet) {
        drvCam.open();
        drvCam.setCameraVideoFormat(drvFmtIdx);
        drvCam.start();

        cameraStatus ds;
        drvCam.getCameraStatus(ds);
        if (ds.status != CAMERA_STATUS::RUNNING) {
            log(dashcam::log::LogLevel::ERROR,
                "driver camera start failed — continuing WITHOUT driver monitoring");
            drvCam.stop(); drvDet.reset();
            drvCam.close();
        } else {
            drvCam.setCaptureEnabled(false);    // branch-only consumer
            drvDet->start();
            const auto deadline =
                std::chrono::steady_clock::now() + kInferenceStartupTimeout;
            while (g_run && drvDet->processedFrameCount() == 0 &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                drvCam.getCameraStatus(ds);
                if (ds.status != CAMERA_STATUS::RUNNING) break;
            }
            if (drvDet->processedFrameCount() == 0) {
                log(dashcam::log::LogLevel::ERROR,
                    "driver path produced no inference result before timeout — "
                    "continuing WITHOUT driver monitoring");
                drvCam.stop(); drvDet->stop(); drvDet.reset();
                drvCam.close();
            } else {
                log(dashcam::log::LogLevel::INFO,
                    "driver inference ready (first result received)");
            }
        }
    }

    // ── recording: compressed UVC passthrough + ASS telemetry sidecar ────────
    // Declared before rec so it outlives the recording pipeline: rec.stopRecording()
    // (below, on shutdown) tears the pipeline down first, so no stream callback can
    // fire into a destroyed server.
    dashcam::network::MediaStreamServer streamSrv;
    dashcam::network::ControlServer     ctrlSrv;

    // ── vehicle telemetry bridge (ESP32-C3 over USB) ─────────────────────────
    // Supplies the position/motion fields the ADAS overlay block leaves at
    // defaults.  Entirely optional: if the C3 is absent the link keeps retrying
    // in its own thread and the overlay simply keeps its defaults, so a missing
    // or unplugged bridge can never stop the dashcam from recording.
    //
    // Declared before rec only for teardown order; it has no dependency on it.
    dashcam::commlink::CommLink bridge;
    {
        dashcam::commlink::CommLinkConfig bcfg;   // by-id discovery, auto-stream
        if (!bridge.open(bcfg, log))
            log(dashcam::log::LogLevel::WARN,
                "telemetry bridge: not present yet — will keep retrying in the background");
        if (!bridge.start())
            log(dashcam::log::LogLevel::ERROR,
                "telemetry bridge: RX thread failed to start — overlay keeps default values");
    }

    dashcam::record::Recorder rec;
    std::string recFile;
    bool recActive = false;
    if (usbInfo) {
        const auto& uFmt = usbInfo->videoFormats[(size_t)usbFmtIdx];
        recFile = footageDir + "/primary_" + localTimestamp() + ".mkv";

        rec.setLogCallback(log);
        rec.setOverlayConfig(cfg.overlay);

        dashcam::record::RecordingFormat rfmt;
        rfmt.v4l2PixFmt = uFmt.pixelFormat;
        rfmt.width      = uFmt.width;
        rfmt.height     = uFmt.height;
        rfmt.fps        = uFmt.frameRate;

        // Start INVALID, so the motion/position corners are dashes until the
        // bridge has actually delivered a sample.  Otherwise OverlayData's
        // built-in placeholder coordinates would be presented as a live fix at
        // the head of every recording.
        //
        // The validity flags say this directly, where the old zero timestamp
        // only implied it — and implied it in a way that depended on
        // staleTimeoutMs being non-zero, which is configurable and can be set to
        // 0 to disable ageing entirely.  In that configuration the placeholders
        // would have been drawn as real.
        dashcam::record::OverlayData od0 = rec.getOverlayData();
        od0.speedValid    = false;
        od0.accelValid    = false;
        od0.positionValid = false;
        od0.headingValid  = false;
        od0.timestampMs   = 0;
        rec.setOverlayData(od0);

        // Live-stream tap (direct precompressed path): fan the recording camera's
        // own compressed frames out to network viewers, no re-encode.  The wire
        // format follows the camera's pixel format.  Must be set BEFORE
        // startRecording() so the tee/appsink branch is built into the pipeline.
        if (cfg.network.streamEnabled && !offline) {
            const bool recMjpeg = uFmt.pixelFormat == V4L2_PIX_FMT_MJPEG;
            dashcam::network::StreamServerConfig sc;
            sc.port        = static_cast<uint16_t>((int)cfg.network.streamPort);
            sc.maxClients  = (int)cfg.network.streamMaxClients;
            sc.bindAddress = (std::string)cfg.network.streamBindAddress;
            sc.wire        = recMjpeg ? dashcam::network::StreamWire::MjpegHttp
                                      : dashcam::network::StreamWire::RawTcp;
            if (streamSrv.start(sc, log)) {
                rec.setCompressedFrameCallback(
                    [&streamSrv](const uint8_t* d, size_t n, bool) { streamSrv.pushFrame(d, n); });
                log(dashcam::log::LogLevel::INFO,
                    std::string("live stream: ") +
                    (recMjpeg ? "open http://<device-ip>:" : "view: ffplay tcp://<device-ip>:") +
                    std::to_string(streamSrv.port()) + (recMjpeg ? "/ in a browser" : ""));
            } else {
                log(dashcam::log::LogLevel::ERROR,
                    "live stream: failed to start — recording without streaming");
            }
        }

        recActive = rec.startRecording(usbInfo->address, rfmt, recFile,
                                       (uint32_t)(int)cfg.recording.recordFps);
        if (!recActive) {
            log(dashcam::log::LogLevel::ERROR,
                "recording failed to start — continuing WITHOUT recording");
            streamSrv.stop();
        }
    }
    if (!recActive && !laneDet && !drvDet) {
        log(dashcam::log::LogLevel::ERROR, "no active subsystems; aborting");
        bridge.stop();   // join before shutdown(): the RX thread holds the log callback
        dashcam::log::shutdown();
        return 1;
    }

    log(dashcam::log::LogLevel::INFO,
        std::string(recActive ? "recording: " + recFile : "recording OFF")
        + (laneDet ? "  | lanes ACTIVE" : "  | lanes OFF")
        + (drvDet  ? "  | driver monitoring ACTIVE" : "  | driver monitoring OFF")
        + "  (SIGINT to stop)");

    // ── remote control + telemetry channel ────────────────────────────────────
    // One TCP connection lets a remote operator re-point the RTP streams at
    // runtime and continuously receive ADAS telemetry (pushed from the loop).
    laneRtpAlive = (laneDet != nullptr && laneSink != nullptr);
    drvRtpAlive  = (drvDet  != nullptr && drvSink  != nullptr);

    dashcam::network::ControlHandler controlHandler =
        [&](const dashcam::network::ControlCommand& cmd) -> std::string {
        if (cmd.verb == "HELP")
            return "commands:\n"
                   "  RTP lane|driver <host|here> [port]   re-point a stream\n"
                   "  RTP lane|driver on|off               pause/resume a stream\n"
                   "  STATUS                               show destinations\n"
                   "  HELP                                 this text\n"
                   "telemetry streams continuously as JSON lines.";

        if (cmd.verb == "STATUS") {
            std::lock_guard<std::mutex> lk(ctrlMtx);
            std::ostringstream os;
            os << "lane rtp=" << laneDestHost << ":" << laneDestPort << " "
               << (laneRtpAlive ? (laneRtpOn ? "on" : "off") : "n/a")
               << " | driver rtp=" << drvDestHost << ":" << drvDestPort << " "
               << (drvRtpAlive ? (drvRtpOn ? "on" : "off") : "n/a");
            return os.str();
        }

        if (cmd.verb == "RTP") {
            if (cmd.args.size() < 2)
                return "ERR usage: RTP lane|driver <host|here|on|off> [port]";
            const bool isLane = (cmd.args[0] == "lane"   || cmd.args[0] == "LANE");
            const bool isDrv  = (cmd.args[0] == "driver" || cmd.args[0] == "DRIVER");
            if (!isLane && !isDrv) return "ERR first arg must be 'lane' or 'driver'";
            const std::string action = cmd.args[1];

            // Pause / resume via the branch valve — gated by the alive flag so a
            // mid-run camera teardown can't race us into a freed valve.
            if (action == "on" || action == "off") {
                const bool enable = (action == "on");
                std::lock_guard<std::mutex> lk(ctrlMtx);
                if (isLane) {
                    if (!laneRtpAlive) return "ERR lane stream not active";
                    laneCam.setBranchEnabled("lane-rtp", enable);
                    laneRtpOn = enable;
                } else {
                    if (!drvRtpAlive) return "ERR driver stream not active";
                    drvCam.setBranchEnabled("driver-rtp", enable);
                    drvRtpOn = enable;
                }
                return std::string("OK ") + (isLane ? "lane" : "driver") + " " + action;
            }

            // Otherwise re-point: action is a destination host, or "here" (the
            // operator's own IP).  Validate the host and parse the port up front,
            // then apply the whole change as ONE synchronized transaction so the
            // actual udpsink destination and the reported bookkeeping can never
            // diverge across concurrent clients.
            const std::string host = (action == "here") ? cmd.clientIp : action;
            if (host.empty())           return "ERR could not resolve host";
            if (!isValidRtpHost(host))  return "ERR invalid host";

            int port;
            { std::lock_guard<std::mutex> lk(ctrlMtx); port = isLane ? laneDestPort : drvDestPort; }
            if (cmd.args.size() >= 3) {
                try { port = std::stoi(cmd.args[2]); }
                catch (...) { return "ERR bad port"; }
                if (port < 1 || port > 65535) return "ERR port out of range";
            }

            {
                // Single transaction: liveness check + sink mutation + bookkeeping
                // all under ctrlMtx.  Teardown (health block) sets *RtpAlive=false
                // under this same lock before stopping the camera, so we either see
                // the stream alive and complete the re-point before teardown runs,
                // or see it dead and reject — never race into a freed valve, and
                // never report a destination the sink didn't actually take.
                std::lock_guard<std::mutex> lk(ctrlMtx);
                const bool alive = isLane ? laneRtpAlive : drvRtpAlive;
                if (!alive)
                    return isLane ? "ERR lane stream not active"
                                  : "ERR driver stream not active";
                GstElement* sink = isLane ? laneSink : drvSink;
                if (!sink) return "ERR that stream has no sink (RTP disabled?)";
                g_object_set(G_OBJECT(sink), "host", host.c_str(), "port", port, NULL);
                if (isLane) { laneDestHost = host; laneDestPort = port; }
                else        { drvDestHost  = host; drvDestPort  = port; }
            }
            log(dashcam::log::LogLevel::INFO,
                std::string("control: ") + (isLane ? "lane" : "driver") +
                " RTP re-pointed -> " + host + ":" + std::to_string(port) +
                " (by " + cmd.clientIp + ")");
            return std::string("OK ") + (isLane ? "lane" : "driver") +
                   " -> " + host + ":" + std::to_string(port);
        }
        return "ERR unknown command (try HELP)";
    };

    if (cfg.network.controlEnabled && !offline) {
        dashcam::network::ControlServerConfig cc;
        cc.port        = static_cast<uint16_t>((int)cfg.network.controlPort);
        cc.maxClients  = (int)cfg.network.controlMaxClients;
        cc.bindAddress = (std::string)cfg.network.controlBindAddress;
        cc.authToken   = (std::string)cfg.network.controlAuthToken;
        // Parse the comma-separated IPv4 allowlist (whitespace-trimmed; blanks dropped).
        {
            const std::string csv = (std::string)cfg.network.controlAllowlist;
            std::istringstream is(csv);
            std::string tok;
            while (std::getline(is, tok, ',')) {
                const size_t a = tok.find_first_not_of(" \t");
                const size_t b = tok.find_last_not_of(" \t");
                if (a != std::string::npos) cc.allowIps.push_back(tok.substr(a, b - a + 1));
            }
        }
        if (ctrlSrv.start(cc, controlHandler, log)) {
            const bool authed = !cc.authToken.empty();
            log(dashcam::log::LogLevel::INFO,
                std::string("remote control: port ") + std::to_string(ctrlSrv.port()) +
                (authed ? " (auth: nonce+HMAC PSK)"
                        : " (UNAUTHENTICATED — set <Network><ControlAuthToken>)") +
                (cc.allowIps.empty() ? "" : " [IP allowlist active]") +
                "  client: python3 src/tools/dashcam_ctl.py <device-ip> " +
                std::to_string(ctrlSrv.port()) +
                (authed ? " [--token-file <path>]" : ""));
        } else {
            log(dashcam::log::LogLevel::ERROR,
                "remote control: failed to start — continuing without it");
        }
    }

    // ── main loop: telemetry clock, health, lane results, driver alerts ──────
    using clock = std::chrono::steady_clock;
    auto secondsSince = [](clock::time_point t) {
        return std::chrono::duration<float>(clock::now() - t).count();
    };

    auto lastStatus = clock::now();
    dashcam::lane::LaneResult lastLane;
    dashcam::driver::DriverStateResult lastDrv;

    // Acute micro-sleep alert hysteresis (fire window from config; see the
    // constants block).  Independent of the long-horizon fatigue score.
    const float acuteAlertSec = (float)cfg.driverScore.acuteAlertSec;
    bool  alertActive = false, faceLost = false;
    auto  drowsySince = clock::now(), naturalSince = clock::now();
    auto  faceLostSince = clock::now();
    bool  drowsyRun = false, naturalRun = false;

    // Fatigue-level transition tracking (score itself lives in the detector).
    auto lastLevel = dashcam::driver::FatigueLevel::OK;
    auto lastFatigueAlarm = clock::now();

    while (g_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Software fatigue-score reset (SIGUSR1; GPIO button later).
        if (g_resetScore) {
            g_resetScore = 0;
            if (drvDet) {
                drvDet->resetScore();
                log(dashcam::log::LogLevel::INFO,
                    "fatigue score reset to a fresh session (SIGUSR1)");
            } else {
                log(dashcam::log::LogLevel::INFO,
                    "fatigue score reset ignored: driver monitoring is OFF");
            }
        }

        if (laneDet) {
            const dashcam::lane::LaneResult lr = laneDet->poll();
            if (lr.valid && lr.sequence != lastLane.sequence &&
                (lr.numLanes != lastLane.numLanes ||
                 lr.currentLaneIndex != lastLane.currentLaneIndex)) {
                log(dashcam::log::LogLevel::INFO,
                    "lane update: lanes=" + std::to_string((int)lr.numLanes)
                    + " ego=" + std::to_string((int)lr.currentLaneIndex));
            }
            if (lr.valid && lr.sequence != lastLane.sequence)
                lastLane = lr;
            // Bridge lane position into the fatigue scorer (the libraries are
            // deliberately decoupled — the app is the only place both exist).
            if (drvDet)
                drvDet->setLaneOffset(lr.lateralOffset,
                                      lr.valid && lr.lateralValid);
        }

        if (drvDet) {
            const dashcam::driver::DriverStateResult dr = drvDet->poll();
            lastDrv = dr;
            if (dr.valid && dr.faceDetected) {
                if (faceLost) {
                    log(dashcam::log::LogLevel::INFO, "driver face reacquired");
                    faceLost = false;
                }
                if (dr.state == dashcam::driver::DriverState::DROWSY) {
                    if (!drowsyRun) { drowsyRun = true; drowsySince = clock::now(); }
                    naturalRun = false;
                    if (!alertActive && secondsSince(drowsySince) >= acuteAlertSec) {
                        alertActive = true;
                        // Alarm hook: GPIO buzzer / MIDI chime goes here.
                        log(dashcam::log::LogLevel::WARN,
                            "DRIVER DROWSY ALERT (p="
                            + std::to_string(dr.drowsyProbability) + ", sustained "
                            + std::to_string((int)acuteAlertSec) + "s)");
                    }
                } else {
                    if (!naturalRun) { naturalRun = true; naturalSince = clock::now(); }
                    drowsyRun = false;
                    if (alertActive && secondsSince(naturalSince) >= kAlertClearSec) {
                        alertActive = false;
                        log(dashcam::log::LogLevel::INFO, "driver drowsy alert cleared");
                    }
                }
                faceLostSince = clock::now();
            } else if (dr.valid) {
                // No face past the detector's own hold window.
                drowsyRun = naturalRun = false;
                if (!faceLost && secondsSince(faceLostSince) >= kNoFaceSec) {
                    faceLost = true;
                    log(dashcam::log::LogLevel::WARN,
                        "driver face not visible for "
                        + std::to_string((int)kNoFaceSec) + "s"
                        + (alertActive ? " (drowsy alert still latched)" : ""));
                }
            }

            // Long-horizon fatigue tier transitions (score-driven, graded).
            using dashcam::driver::FatigueLevel;
            if (dr.valid && dr.fatigueLevel != lastLevel) {
                const std::string msg =
                    std::string("fatigue level ") + levelStr(lastLevel) + " -> "
                    + levelStr(dr.fatigueLevel)
                    + " (score=" + std::to_string((int)dr.fatigueScore)
                    + "/" + std::to_string((int)dr.fatigueCap) + ")";
                if (dr.fatigueLevel == FatigueLevel::FATIGUE) {
                    // Strong alarm hook (repeating chime / voice prompt) here.
                    log(dashcam::log::LogLevel::ERROR,
                        "DRIVER FATIGUE — high-confidence, sustained: pull "
                        "over when safe. " + msg);
                    lastFatigueAlarm = clock::now();
                } else if (dr.fatigueLevel > lastLevel)
                    log(dashcam::log::LogLevel::WARN, msg);
                else
                    log(dashcam::log::LogLevel::INFO, msg + " (recovering)");
                lastLevel = dr.fatigueLevel;
            }
            // Re-raise the FATIGUE alarm while it stays latched (a missed
            // one-shot is no alert).
            if (dr.valid && dr.fatigueLevel == FatigueLevel::FATIGUE &&
                secondsSince(lastFatigueAlarm) >= kFatigueRealertSec) {
                lastFatigueAlarm = clock::now();
                log(dashcam::log::LogLevel::ERROR,
                    "DRIVER FATIGUE — still fatigued (score="
                    + std::to_string((int)dr.fatigueScore)
                    + "): pull over when safe.");
            }
        }

        // ── telemetry: ADAS overlays into the recording + push to remote ──────
        // Built from the freshest lane + driver results this iteration.  Motion
        // and position come from the ESP32-C3 bridge (OBD2 + GPS via the MKR
        // Zero); when it goes quiet the recorder draws dashes instead (below).
        {
            dashcam::record::OverlayData od =
                recActive ? rec.getOverlayData() : dashcam::record::OverlayData{};

            // Vehicle telemetry.  MOTION and POSITION are gated separately, on
            // their own source's validity flag — not on whether the bridge is
            // alive.  A live bridge says the C3 and the MKR are talking; it says
            // nothing about whether the ECU answered or the receiver has a fix,
            // and those fail independently of the link and of each other.
            //
            // Gating both on bridge liveness (as this did) meant a healthy link
            // carrying a dead source kept that source's last values in od — od
            // is inherited from the previous sample — and went on stamping them
            // current.  A frozen speed or a minutes-old position burned into a
            // recording and presented as live is false evidence, which is the one
            // output a dashcam must never produce.
            // One clock read per tick.  Every per-source stamp below refers to the
            // same sample, so reading the clock separately for each of them was
            // both wasteful and slightly wrong — the stamps could differ by the
            // time the block took to run.
            const int64_t tickMs = epochMs();
            const bool bridgeFresh = !bridge.isStale(kBridgeFreshnessMs);
            const auto t = bridge.telemetry();

            // OBD2_VALID is the master's own freshness verdict on the ECU: it
            // clears when no reading has arrived within the master's window, so
            // it distinguishes "ECU answering" from "MKR alive but ECU silent".
            const bool obdLive = bridgeFresh && (t.flags & hostproto::TLM_FLAG_OBD2_VALID) != 0;
            const bool fixLive = bridgeFresh && t.fixValid;

            // Speed and acceleration are tracked SEPARATELY, not as one "motion"
            // block.  Speed can come from either source; acceleration only ever
            // comes from the ECU.  A shared flag therefore certified a stale
            // acceleration whenever a live GNSS fix refreshed speed with the ECU
            // dead — the inherited value from the previous frame, rendered as
            // current.  Two flags cannot do that.
            if (fixLive && !std::isnan(t.gpsSpeedKmh)) {
                od.speedKmh         = t.gpsSpeedKmh;
                od.speedValid       = true;
                od.speedTimestampMs = tickMs;
            } else if (obdLive && !std::isnan(t.speed)) {
                od.speedKmh         = t.speed;
                od.speedValid       = true;
                od.speedTimestampMs = tickMs;
            } else {
                // Cleared, not inherited.  Leaving the previous number in place
                // relies on every downstream consumer checking the flag, and one
                // that forgets renders a stale speed indistinguishable from a
                // live one.  A sentinel makes that mistake impossible.
                od.speedValid = false;
                od.speedKmh   = 0.0f;
            }

            // Acceleration is estimated on the MKR Zero (smoothed speed,
            // differentiated, jerk-limited) rather than derived here: this
            // stream repeats each 1 km/h reading several times, so a derivative
            // taken on this side would be spikes, not motion.  NaN means the
            // estimator has not warmed up yet.
            if (obdLive && !std::isnan(t.accel)) {
                od.accelerationMs2  = t.accel;
                od.accelValid       = true;
                od.accelTimestampMs = tickMs;
            } else {
                od.accelValid      = false;
                od.accelerationMs2 = 0.0f;
            }

            if (fixLive && !std::isnan(t.latitude) && !std::isnan(t.longitude)) {
                od.latitude  = t.latitude;
                od.longitude = t.longitude;
                if (!std::isnan(t.altitude)) od.altitudeM = t.altitude;

                od.positionValid       = true;
                od.positionTimestampMs = tickMs;
            } else {
                od.positionValid = false;
                od.latitude      = 0.0;
                od.longitude     = 0.0;
                od.altitudeM     = 0.0;
            }

            // Heading is tracked SEPARATELY from the coordinates, even though
            // both come from the receiver, because they do not fail together.
            // The master sends NaN whenever it judged the course untrustworthy —
            // below ~5 km/h a receiver reports a direction that wanders the whole
            // circle — and that happens at every stop, with the fix itself
            // perfectly good.  Folding heading into positionValid therefore
            // certified the last travelled heading, or before any fix at all
            // OverlayData's 90.0f placeholder, as a live reading every time the
            // vehicle stopped.
            if (fixLive && !std::isnan(t.heading)) {
                od.headingDeg         = t.heading;
                od.headingValid       = true;
                od.headingTimestampMs = tickMs;
            } else {
                od.headingValid = false;
                od.headingDeg   = 0.0f;
            }

            // The JSON sidecar's "t" field.  Advanced EVERY tick, on purpose and
            // unlike the per-source stamps above: it is the time this SAMPLE was
            // taken, and the sample always includes fresh ADAS state even when
            // every vehicle source is dead.  Leaving it behind (or at the zero it
            // is seeded with) would have published live lane and fatigue results
            // stamped with a stale — or 1970 — timestamp.
            //
            // It is deliberately NOT used to age the motion or position fields;
            // that is what speedValid/accelValid/positionValid and their own
            // stamps are for.
            od.timestampMs = tickMs;

            // Build the ADAS block FRESH from this tick's results — never inherit
            // ADAS fields from the previous OverlayData.  A source contributes only
            // when it is both configured AND has a fresh valid sample; otherwise its
            // fields are reset to sentinels so a warming-up or torn-down detector can
            // never publish a fabricated "fatigue=100/face=true" or a stale reading.
            const bool laneValid =
                laneDet != nullptr && lastLane.valid &&
                laneDet->hasFreshResult(kLaneFreshnessMs);
            const bool drvValid =
                drvDet != nullptr && lastDrv.valid &&
                drvDet->hasFreshResult(kDriverFreshnessMs);
            od.laneValid   = laneValid;
            od.driverValid = drvValid;
            od.adasValid   = laneValid || drvValid;
            if (laneValid) {
                od.laneCount       = lastLane.numLanes;
                od.egoLaneIndex    = lastLane.currentLaneIndex;
                od.laneOffset      = lastLane.lateralOffset;
                od.laneOffsetValid = lastLane.lateralValid;
            } else {
                od.laneCount       = 0;
                od.egoLaneIndex    = -1;
                od.laneOffset      = 0.0f;
                od.laneOffsetValid = false;
            }
            if (drvValid) {
                od.fatigueScore = lastDrv.fatigueScore;
                od.fatigueLevel = static_cast<int>(lastDrv.fatigueLevel);
                od.driverDrowsy = lastDrv.faceDetected &&
                                  lastDrv.state == dashcam::driver::DriverState::DROWSY;
                od.faceDetected = lastDrv.faceDetected;
            } else {
                od.fatigueScore = 100.0f;
                od.fatigueLevel = 0;
                od.driverDrowsy = false;
                od.faceDetected = false;
            }
            if (recActive) rec.setOverlayData(od);

            if (ctrlSrv.isRunning() && ctrlSrv.clientCount() > 0) {
                std::string lh, dh; int lp, dp;
                { std::lock_guard<std::mutex> lk(ctrlMtx);
                  lh = laneDestHost; lp = laneDestPort;
                  dh = drvDestHost;  dp = drvDestPort; }
                ctrlSrv.pushTelemetry(makeTelemetryJson(
                    od, laneDet != nullptr, drvDet != nullptr, alertActive,
                    lh, lp, dh, dp));
            }
        }

        auto now = clock::now();
        if (now - lastStatus >= std::chrono::seconds(5)) {
            lastStatus = now;
            if (recActive && !rec.isRecording()) {
                log(dashcam::log::LogLevel::ERROR,
                    "recording no longer healthy — disabling recorder and "
                    "continuing remaining subsystems");
                rec.stopRecording();
                streamSrv.stop();
                recActive = false;
            }
            if (laneDet) {
                cameraStatus ls;
                laneCam.getCameraStatus(ls);
                if (ls.status != CAMERA_STATUS::RUNNING ||
                    !laneDet->hasFreshResult(kLaneFreshnessMs)) {
                    log(dashcam::log::LogLevel::ERROR,
                        ls.status != CAMERA_STATUS::RUNNING
                            ? "IMX296 no longer RUNNING — lane detection lost"
                            : "lane inference results became stale — lane "
                              "detection lost");
                    // Mark the stream dead BEFORE teardown so a concurrent
                    // control command can't call setBranchEnabled on a freed valve.
                    { std::lock_guard<std::mutex> lk(ctrlMtx); laneRtpAlive = false; }
                    laneCam.stop(); laneDet->stop(); laneDet.reset();
                    laneCam.close();
                } else {
                    log(dashcam::log::LogLevel::DEBUG,
                        "lane: lanes=" + std::to_string((int)lastLane.numLanes)
                        + " ego=" + std::to_string((int)lastLane.currentLaneIndex));
                }
            }
            if (drvDet) {
                cameraStatus ds;
                drvCam.getCameraStatus(ds);
                if (ds.status != CAMERA_STATUS::RUNNING ||
                    !drvDet->hasFreshResult(kDriverFreshnessMs)) {
                    log(dashcam::log::LogLevel::ERROR,
                        ds.status != CAMERA_STATUS::RUNNING
                            ? "driver camera no longer RUNNING — driver "
                              "monitoring lost"
                            : "driver inference results became stale — driver "
                              "monitoring lost");
                    { std::lock_guard<std::mutex> lk(ctrlMtx); drvRtpAlive = false; }
                    drvCam.stop(); drvDet->stop(); drvDet.reset();
                    drvCam.close();
                } else {
                    log(dashcam::log::LogLevel::DEBUG,
                        std::string("driver: ")
                        + (!lastDrv.valid ? "warming up"
                           : !lastDrv.faceDetected ? "no face"
                           : "p(drowsy)=" + std::to_string(lastDrv.drowsyProbability))
                        + (lastDrv.valid
                           ? "  score=" + std::to_string((int)lastDrv.fatigueScore)
                             + "/" + std::to_string((int)lastDrv.fatigueCap)
                             + " " + levelStr(lastDrv.fatigueLevel)
                           : "")
                        + (alertActive ? " [ALERT]" : ""));
                }
            }
            if (!recActive && !laneDet && !drvDet) {
                log(dashcam::log::LogLevel::ERROR,
                    "all subsystems lost; stopping");
                break;
            }
        }
    }

    // ── graceful shutdown: EOS-finalise the MKV, close the sidecar ───────────
    log(dashcam::log::LogLevel::INFO, "shutting down (signal or subsystem loss)");
    ctrlSrv.stop();                       // FIRST: join client threads so no command
                                          // handler can touch the cameras mid-teardown.
    bridge.stop();                        // join the RX thread while the logger is still
                                          // up — liblog requires every thread that can
                                          // call the log callback to be joined before
                                          // shutdown(), and the bridge logs reconnects.
    if (recActive) rec.stopRecording();   // stops the pipeline → no more stream callbacks
    streamSrv.stop();                     // then disconnect viewers (safe: no callbacks in flight)
    if (laneDet) {
        laneCam.stop();
        laneDet->stop();
        laneCam.close();
    }
    if (drvDet) {
        drvCam.stop();      // flushing appsink → inference thread drains
        drvDet->stop();     // joins inference thread
        drvCam.close();
    }
    // Release the kept udpsink refs now the pipelines that owned them are gone.
    if (laneSink) { gst_object_unref(laneSink); laneSink = nullptr; }
    if (drvSink)  { gst_object_unref(drvSink);  drvSink  = nullptr; }

    if (recActive) {
        auto sizeKb = [](const std::string& f) -> long {
            return fs::exists(f) ? (long)(fs::file_size(f) / 1024) : 0;
        };
        const std::string assFile = recFile.substr(0, recFile.size() - 4) + ".ass";
        log(dashcam::log::LogLevel::INFO,
            "stopped. primary " + std::to_string(sizeKb(recFile)) + " KB, telemetry "
            + std::to_string(sizeKb(assFile)) + " KB");
    } else {
        log(dashcam::log::LogLevel::INFO, "stopped.");
    }
    dashcam::log::shutdown();
    return 0;
}
