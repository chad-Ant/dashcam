#include "libcamera.h"
#include "libconfig.h"
#include <algorithm>
#include <climits>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <pugixml.hpp>
#include <unistd.h>

// ─── file-local helpers ───────────────────────────────────────────────────────

namespace {

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

// "<name>" directory next to the running executable (independent of the cwd),
// e.g. the bin/build_<ts>/<name> of the build this binary came from.  Falls back
// to a cwd-relative "<name>" if the executable path can't be read.
static std::string exeRelativeDirImpl(const std::string& name) {
    char buf[PATH_MAX];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return name;
    buf[n] = '\0';
    std::string exe(buf);
    std::string::size_type slash = exe.find_last_of('/');
    std::string base = (slash == std::string::npos) ? std::string(".") : exe.substr(0, slash);
    return base + "/" + name;
}

static std::string exeRelativeConfigDir() { return exeRelativeDirImpl("config"); }

// True if `dir` can be created and actually written to.  Creates it if missing,
// then writes and removes a probe file — this catches a removed SD card / stale or
// read-only mount (ENOENT / EROFS / EIO) that a plain exists() check would miss.
static bool dirWritable(const std::string& dir) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);   // no error if it already exists
    const std::string probe = dir + "/.dashcam_write_probe";
    FILE* f = std::fopen(probe.c_str(), "w");
    if (!f) return false;
    bool ok = (std::fputc('x', f) != EOF);
    if (std::fclose(f) != 0) ok = false;            // deferred write errors surface here
    std::remove(probe.c_str());
    return ok;
}

// Return the XML attribute string for a ConfigDataType.
static const char* typeName(dashcam::config::ConfigDataType dt) {
    using dashcam::config::ConfigDataType;
    switch (dt) {
        case ConfigDataType::Int:    return "int";
        case ConfigDataType::Float:  return "float";
        case ConfigDataType::Bool:   return "bool";
        case ConfigDataType::String: return "string";
    }
    return "string";
}

// ─── readVar ─────────────────────────────────────────────────────────────────
// Read a ConfigVar<T> from the child element whose tag matches v.name().
// Numeric values outside [min, max] are clamped; a WARN is emitted.

template<typename T>
static void readVar(pugi::xml_node parent, dashcam::config::ConfigVar<T>& v,
                    const dashcam::log::LogCallback& log) {
    pugi::xml_node n = parent.child(v.name().c_str());
    if (!n) return;

    T parsed;
    if constexpr (std::is_same_v<T, int>)
        parsed = n.text().as_int(static_cast<int>(v.defaultValue()));
    else if constexpr (std::is_same_v<T, float>)
        parsed = static_cast<float>(n.text().as_double(static_cast<double>(v.defaultValue())));
    else if constexpr (std::is_same_v<T, bool>)
        parsed = n.text().as_bool(static_cast<bool>(v.defaultValue()));
    else
        parsed = std::string(n.text().as_string(v.defaultValue().c_str()));

    if (!v.set(parsed))
        doLog(log, dashcam::log::LogLevel::WARN,
              "config '%s': value out of range, clamped to [min, max]",
              v.name().c_str());
}

// ─── writeVar ────────────────────────────────────────────────────────────────
// Append a child element named v.name() with metadata attributes and text value.
//
// For numeric types: type, min, max, step, default, description are attributes.
// For bool:          type, default, description.
// For string:        type, default, description.

template<typename T>
static void writeVar(pugi::xml_node parent, const dashcam::config::ConfigVar<T>& v) {
    pugi::xml_node n = parent.append_child(v.name().c_str());
    n.append_attribute("type").set_value(typeName(v.datatype()));
    n.append_attribute("description").set_value(v.description().c_str());

    if constexpr (std::is_same_v<T, int>) {
        // Write ints as ints so pugixml formats them exactly (e.g. "8000").
        n.append_attribute("min").set_value(v.minValue());
        n.append_attribute("max").set_value(v.maxValue());
        n.append_attribute("step").set_value(v.step());
        n.append_attribute("default").set_value(v.defaultValue());
        n.text().set(static_cast<int>(v));
    } else if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {  // float
        // pugixml formats a bare double at ~17 significant figures, so 0.85f becomes
        // "0.85000002384185791" — noise that defeats the self-documenting config.
        // Config granularity (steps ≥ 0.01) never needs more than a few digits, so
        // write floats at a modest precision to keep the file clean and diff-friendly.
        constexpr int kPrec = 6;
        n.append_attribute("min").set_value(static_cast<double>(v.minValue()), kPrec);
        n.append_attribute("max").set_value(static_cast<double>(v.maxValue()), kPrec);
        n.append_attribute("step").set_value(static_cast<double>(v.step()), kPrec);
        n.append_attribute("default").set_value(static_cast<double>(v.defaultValue()), kPrec);
        n.text().set(static_cast<double>(static_cast<T>(v)), kPrec);
    } else if constexpr (std::is_same_v<T, bool>) {
        n.append_attribute("default").set_value(static_cast<bool>(v.defaultValue()));
        n.text().set(static_cast<bool>(v));
    } else {
        n.append_attribute("default").set_value(v.defaultValue().c_str());
        n.text().set(static_cast<const std::string&>(v).c_str());
    }
}

} // namespace

// ─── parseValueType (AttributeDictionary) ────────────────────────────────────

namespace {

dashcam::camera::AttributeValueType parseValueType(const char* str) {
    using dashcam::camera::AttributeValueType;
    if (!str || !*str) return AttributeValueType::String;
    std::string s(str);
    if (s == "int")            return AttributeValueType::Int;
    if (s == "float")          return AttributeValueType::Float;
    if (s == "bool")           return AttributeValueType::Bool;
    if (s == "bool_from_zero") return AttributeValueType::BoolFromZero;
    if (s == "range_string")   return AttributeValueType::RangeString;
    return AttributeValueType::String;
}

} // namespace

// ─── dashcam::config ──────────────────────────────────────────────────────────

namespace dashcam::config {

namespace {

// ── section parsers ──────────────────────────────────────────────────────────

static void parseEncoder(pugi::xml_node node, EncoderConfig& enc,
                         const dashcam::log::LogCallback& log) {
    readVar(node, enc.bitrate,     log);
    readVar(node, enc.speedPreset, log);
    readVar(node, enc.keyIntMax,   log);
    readVar(node, enc.tune,        log);
}

static void parseOverlay(pugi::xml_node node, OverlayConfig& ovl,
                         const dashcam::log::LogCallback& log) {
    readVar(node, ovl.enabled,           log);
    readVar(node, ovl.backgroundOpacity, log);
    readVar(node, ovl.fontSize,          log);
    readVar(node, ovl.fontFace,          log);
    readVar(node, ovl.labelPadX,         log);
    readVar(node, ovl.labelPadY,         log);
    readVar(node, ovl.subtitleRateHz,    log);
    readVar(node, ovl.staleTimeoutMs,    log);
}

static void parseCamera(pugi::xml_node node, CameraConfig& cam,
                        const dashcam::log::LogCallback& log) {
    cam.name = node.attribute("name").as_string(cam.name.c_str());
    cam.type = node.attribute("type").as_string(cam.type.c_str());

    readVar(node, cam.enabled,     log);
    readVar(node, cam.device,      log);
    readVar(node, cam.sensorId,    log);
    readVar(node, cam.formatIndex, log);
    readVar(node, cam.outWidth,    log);
    readVar(node, cam.outHeight,   log);
    readVar(node, cam.outFps,      log);

    if (auto attrs = node.child("Attributes")) {
        for (auto attr : attrs.children("Attribute")) {
            CameraAttributeInfo ai;
            ai.name     = attr.attribute("name").as_string();
            ai.writable = attr.attribute("writable").as_bool();
            ai.readable = attr.attribute("readable").as_bool();
            ai.minValue = attr.attribute("min").as_float();
            ai.maxValue = attr.attribute("max").as_float();
            ai.step     = attr.attribute("step").as_float();
            ai.menuOptions = attr.attribute("menu").as_string();
            if (!ai.name.empty())
                cam.attributeInfo.push_back(std::move(ai));
        }
    }

    if (auto caps = node.child("Capabilities")) {
        for (auto cap : caps.children("Capability")) {
            const char* key = cap.attribute("name").value();
            const char* val = cap.attribute("value").value();
            if (key && *key)
                cam.capabilities[key] = val;
        }
    }
}

static void parseSystem(pugi::xml_node node, SystemConfig& sys,
                        const dashcam::log::LogCallback& log) {
    readVar(node, sys.footagePath,  log);
    readVar(node, sys.warmupFrames, log);
}

static void parsePipeline(pugi::xml_node node, PipelineConfig& p,
                          const dashcam::log::LogCallback& log) {
    readVar(node, p.captureTimeoutMs,     log);
    readVar(node, p.stateChangeTimeoutMs, log);
    readVar(node, p.eosTimeoutMs,         log);
    readVar(node, p.captureQueueDepth,    log);
    readVar(node, p.appsinkMaxBuffers,    log);
    readVar(node, p.branchQueueDepth,     log);
}

static void parseLog(pugi::xml_node node, LogConfig& l,
                     const dashcam::log::LogCallback& log) {
    readVar(node, l.queueSize,     log);
    readVar(node, l.rotateSizeKb,  log);
    readVar(node, l.rotateFiles,   log);
    readVar(node, l.flushEverySec, log);
    readVar(node, l.level,         log);
    readVar(node, l.flushOn,       log);
}

static void parseRecording(pugi::xml_node node, RecordingConfig& r,
                           const dashcam::log::LogCallback& log) {
    readVar(node, r.recordFps,    log);
    readVar(node, r.queueDepth,   log);
    readVar(node, r.recordWidth,  log);
    readVar(node, r.recordHeight, log);
}

static void parseDetection(pugi::xml_node node, DetectionConfig& d,
                           const dashcam::log::LogCallback& log) {
    readVar(node, d.laneEnginePath,    log);
    readVar(node, d.laneTargetHz,      log);
    readVar(node, d.laneBranchMaxFps,  log);
    readVar(node, d.laneInputCropTop,  log);
    readVar(node, d.laneInputCropBottom, log);
    readVar(node, d.laneReferenceY,    log);
    readVar(node, d.signTargetHz,      log);
    readVar(node, d.signConfThreshold, log);
    readVar(node, d.signNmsThreshold,  log);
    readVar(node, d.driverEnginePath,  log);
    readVar(node, d.driverTargetHz,    log);
    readVar(node, d.driverBranchMaxFps, log);
    readVar(node, d.driverDrowsyThreshold, log);
    readVar(node, d.driverFaceDetection, log);
    readVar(node, d.driverFaceModelPath, log);
    readVar(node, d.driverFaceScore,     log);
    readVar(node, d.driverFaceDetectScale, log);
}

static void parseNetwork(pugi::xml_node node, NetworkConfig& n,
                         const dashcam::log::LogCallback& log) {
    readVar(node, n.wifiConnectEnabled,  log);
    readVar(node, n.wifiSsid,            log);
    readVar(node, n.wifiTimeoutSec,      log);
    readVar(node, n.wifiRequireInternet, log);
    readVar(node, n.timeSyncEnabled,     log);
    readVar(node, n.ntpServer,           log);
    readVar(node, n.ntpPort,             log);
    readVar(node, n.ntpTimeoutMs,        log);
    readVar(node, n.ntpRetries,          log);
    readVar(node, n.ntpStepClock,        log);
    readVar(node, n.ntpStepThresholdSec, log);
    readVar(node, n.streamEnabled,       log);
    readVar(node, n.streamPort,          log);
    readVar(node, n.streamMaxClients,    log);
    readVar(node, n.rtpEnabled,          log);
    readVar(node, n.rtpHost,             log);
    readVar(node, n.rtpPort,             log);
    readVar(node, n.rtpBitrateKbps,      log);
}

static void parseDriverScore(pugi::xml_node node, DriverScoreConfig& s,
                             const dashcam::log::LogCallback& log) {
    readVar(node, s.scoreInitial,       log);
    readVar(node, s.scoreUpper,         log);
    readVar(node, s.scoreLower,         log);
    readVar(node, s.drowsyChunkSec,     log);
    readVar(node, s.drowsyChunkPenalty, log);
    readVar(node, s.awakeChunkSec,      log);
    readVar(node, s.awakeChunkReward,   log);
    readVar(node, s.laneDepartThresh,   log);
    readVar(node, s.laneReturnSec,      log);
    readVar(node, s.laneDriftPenalty,   log);
    readVar(node, s.capDecayPerHour,    log);
    readVar(node, s.capDecayFloor,      log);
    readVar(node, s.cautionScore,       log);
    readVar(node, s.warningScore,       log);
    readVar(node, s.fatigueScore,       log);
    readVar(node, s.fatigueSustainSec,  log);
    readVar(node, s.acuteAlertSec,      log);
    readVar(node, s.noFaceFreezes,      log);
}

// ── section writers ──────────────────────────────────────────────────────────

static void writeEncoder(pugi::xml_node parent, const EncoderConfig& enc) {
    pugi::xml_node n = parent.append_child("Encoder");
    writeVar(n, enc.bitrate);
    writeVar(n, enc.speedPreset);
    writeVar(n, enc.keyIntMax);
    writeVar(n, enc.tune);
}

static void writeOverlay(pugi::xml_node parent, const OverlayConfig& ovl) {
    pugi::xml_node n = parent.append_child("Overlay");
    writeVar(n, ovl.enabled);
    writeVar(n, ovl.backgroundOpacity);
    writeVar(n, ovl.fontSize);
    writeVar(n, ovl.fontFace);
    writeVar(n, ovl.labelPadX);
    writeVar(n, ovl.labelPadY);
    writeVar(n, ovl.subtitleRateHz);
    writeVar(n, ovl.staleTimeoutMs);
}

static void writeCamera(pugi::xml_node parent, const CameraConfig& cam) {
    pugi::xml_node n = parent.append_child("Camera");
    n.append_attribute("name").set_value(cam.name.c_str());
    n.append_attribute("type").set_value(cam.type.c_str());

    writeVar(n, cam.enabled);
    writeVar(n, cam.device);
    writeVar(n, cam.sensorId);
    writeVar(n, cam.formatIndex);
    writeVar(n, cam.outWidth);
    writeVar(n, cam.outHeight);
    writeVar(n, cam.outFps);

    if (!cam.attributeInfo.empty()) {
        pugi::xml_node attrs = n.append_child("Attributes");
        for (const auto& ai : cam.attributeInfo) {
            pugi::xml_node a = attrs.append_child("Attribute");
            a.append_attribute("name").set_value(ai.name.c_str());
            a.append_attribute("writable").set_value(ai.writable);
            a.append_attribute("readable").set_value(ai.readable);
            a.append_attribute("min").set_value(static_cast<double>(ai.minValue));
            a.append_attribute("max").set_value(static_cast<double>(ai.maxValue));
            a.append_attribute("step").set_value(static_cast<double>(ai.step));
            if (!ai.menuOptions.empty())
                a.append_attribute("menu").set_value(ai.menuOptions.c_str());
        }
    }

    if (!cam.capabilities.empty()) {
        pugi::xml_node caps = n.append_child("Capabilities");
        for (const auto& [key, val] : cam.capabilities) {
            pugi::xml_node cap = caps.append_child("Capability");
            cap.append_attribute("name").set_value(key.c_str());
            cap.append_attribute("value").set_value(val.c_str());
        }
    }
}

static void writeSystem(pugi::xml_node parent, const SystemConfig& sys) {
    pugi::xml_node n = parent.append_child("System");
    writeVar(n, sys.footagePath);
    writeVar(n, sys.warmupFrames);
}

static void writePipeline(pugi::xml_node parent, const PipelineConfig& p) {
    pugi::xml_node n = parent.append_child("Pipeline");
    writeVar(n, p.captureTimeoutMs);
    writeVar(n, p.stateChangeTimeoutMs);
    writeVar(n, p.eosTimeoutMs);
    writeVar(n, p.captureQueueDepth);
    writeVar(n, p.appsinkMaxBuffers);
    writeVar(n, p.branchQueueDepth);
}

static void writeLog(pugi::xml_node parent, const LogConfig& l) {
    pugi::xml_node n = parent.append_child("Log");
    writeVar(n, l.queueSize);
    writeVar(n, l.rotateSizeKb);
    writeVar(n, l.rotateFiles);
    writeVar(n, l.flushEverySec);
    writeVar(n, l.level);
    writeVar(n, l.flushOn);
}

static void writeRecording(pugi::xml_node parent, const RecordingConfig& r) {
    pugi::xml_node n = parent.append_child("Recording");
    writeVar(n, r.recordFps);
    writeVar(n, r.queueDepth);
    writeVar(n, r.recordWidth);
    writeVar(n, r.recordHeight);
}

static void writeDetection(pugi::xml_node parent, const DetectionConfig& d) {
    pugi::xml_node n = parent.append_child("Detection");
    writeVar(n, d.laneEnginePath);
    writeVar(n, d.laneTargetHz);
    writeVar(n, d.laneBranchMaxFps);
    writeVar(n, d.laneInputCropTop);
    writeVar(n, d.laneInputCropBottom);
    writeVar(n, d.laneReferenceY);
    writeVar(n, d.signTargetHz);
    writeVar(n, d.signConfThreshold);
    writeVar(n, d.signNmsThreshold);
    writeVar(n, d.driverEnginePath);
    writeVar(n, d.driverTargetHz);
    writeVar(n, d.driverBranchMaxFps);
    writeVar(n, d.driverDrowsyThreshold);
    writeVar(n, d.driverFaceDetection);
    writeVar(n, d.driverFaceModelPath);
    writeVar(n, d.driverFaceScore);
    writeVar(n, d.driverFaceDetectScale);
}

static void writeNetwork(pugi::xml_node parent, const NetworkConfig& n) {
    pugi::xml_node node = parent.append_child("Network");
    writeVar(node, n.wifiConnectEnabled);
    writeVar(node, n.wifiSsid);
    writeVar(node, n.wifiTimeoutSec);
    writeVar(node, n.wifiRequireInternet);
    writeVar(node, n.timeSyncEnabled);
    writeVar(node, n.ntpServer);
    writeVar(node, n.ntpPort);
    writeVar(node, n.ntpTimeoutMs);
    writeVar(node, n.ntpRetries);
    writeVar(node, n.ntpStepClock);
    writeVar(node, n.ntpStepThresholdSec);
    writeVar(node, n.streamEnabled);
    writeVar(node, n.streamPort);
    writeVar(node, n.streamMaxClients);
    writeVar(node, n.rtpEnabled);
    writeVar(node, n.rtpHost);
    writeVar(node, n.rtpPort);
    writeVar(node, n.rtpBitrateKbps);
}

static void writeDriverScore(pugi::xml_node parent, const DriverScoreConfig& s) {
    pugi::xml_node n = parent.append_child("DriverScore");
    writeVar(n, s.scoreInitial);
    writeVar(n, s.scoreUpper);
    writeVar(n, s.scoreLower);
    writeVar(n, s.drowsyChunkSec);
    writeVar(n, s.drowsyChunkPenalty);
    writeVar(n, s.awakeChunkSec);
    writeVar(n, s.awakeChunkReward);
    writeVar(n, s.laneDepartThresh);
    writeVar(n, s.laneReturnSec);
    writeVar(n, s.laneDriftPenalty);
    writeVar(n, s.capDecayPerHour);
    writeVar(n, s.capDecayFloor);
    writeVar(n, s.cautionScore);
    writeVar(n, s.warningScore);
    writeVar(n, s.fatigueScore);
    writeVar(n, s.fatigueSustainSec);
    writeVar(n, s.acuteAlertSec);
    writeVar(n, s.noFaceFreezes);
}

} // namespace

// ── ConfigReader ──────────────────────────────────────────────────────────────

bool ConfigReader::load(const std::string& filePath, AppConfig& config,
                        const dashcam::log::LogCallback& log) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filePath.c_str());
    if (!result) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "cannot parse '%s': %s", filePath.c_str(), result.description());
        return false;
    }

    pugi::xml_node root = doc.child("DashcamConfig");
    if (!root) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "missing <DashcamConfig> root in '%s'", filePath.c_str());
        return false;
    }

    if (auto enc = root.child("Encoder"))   parseEncoder  (enc, config.encoder,   log);
    if (auto ovl = root.child("Overlay"))   parseOverlay  (ovl, config.overlay,   log);
    if (auto sys = root.child("System"))    parseSystem   (sys, config.system,    log);
    if (auto pl  = root.child("Pipeline"))  parsePipeline (pl,  config.pipeline,  log);
    if (auto rec = root.child("Recording")) parseRecording(rec, config.recording, log);
    if (auto det = root.child("Detection")) parseDetection(det, config.detection, log);
    if (auto ds  = root.child("DriverScore"))
        parseDriverScore(ds, config.driverScore, log);
    if (auto lg  = root.child("Log"))       parseLog      (lg,  config.log,       log);
    if (auto net = root.child("Network"))   parseNetwork  (net, config.network,   log);

    if (auto cams = root.child("Cameras")) {
        config.cameras.clear();
        for (auto cam : cams.children("Camera")) {
            CameraConfig cc;
            parseCamera(cam, cc, log);
            config.cameras.push_back(std::move(cc));
        }
    }

    return true;
}

bool ConfigReader::save(const std::string& filePath, const AppConfig& config,
                        const dashcam::log::LogCallback& log) {
    pugi::xml_document doc;

    pugi::xml_node decl = doc.prepend_child(pugi::node_declaration);
    decl.append_attribute("version").set_value("1.0");
    decl.append_attribute("encoding").set_value("UTF-8");

    pugi::xml_node root = doc.append_child("DashcamConfig");

    writeEncoder(root, config.encoder);
    writeOverlay(root, config.overlay);

    pugi::xml_node cams = root.append_child("Cameras");
    for (const auto& cam : config.cameras)
        writeCamera(cams, cam);

    writeSystem(root, config.system);
    writePipeline(root, config.pipeline);
    writeRecording(root, config.recording);
    writeDetection(root, config.detection);
    writeDriverScore(root, config.driverScore);
    writeLog(root, config.log);
    writeNetwork(root, config.network);

    if (!doc.save_file(filePath.c_str(), "  ")) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "cannot write '%s'", filePath.c_str());
        return false;
    }
    return true;
}

bool ConfigReader::loadOrCreate(const std::string& filePath, AppConfig& config,
                                const dashcam::log::LogCallback& log) {
    std::error_code ec;
    if (std::filesystem::exists(filePath, ec))
        return load(filePath, config, log);

    // Missing: create the parent directory and seed a default-valued config.
    std::filesystem::path p(filePath);
    if (p.has_parent_path())
        std::filesystem::create_directories(p.parent_path(), ec);

    AppConfig defaults;                 // built-in defaults
    if (!save(filePath, defaults, log)) // save() reports its own error
        return false;

    doLog(log, dashcam::log::LogLevel::INFO,
          "config '%s' not found — created with default values", filePath.c_str());
    config = defaults;
    return true;
}

std::string configDir() { return exeRelativeConfigDir(); }

std::string exeRelativeDir(const std::string& name) { return exeRelativeDirImpl(name); }

std::string resolveStorageDir(const std::string& preferred,
                              const std::string& fallbackName,
                              const dashcam::log::LogCallback& log) {
    if (dirWritable(preferred))
        return preferred;

    const std::string fallback = exeRelativeDirImpl(fallbackName);
    doLog(log, dashcam::log::LogLevel::WARN,
          "storage '%s' unavailable (SD card removed?) — falling back to build-local '%s'",
          preferred.c_str(), fallback.c_str());
    // Best-effort create; if even this fails there is nothing writable to use.
    std::error_code ec;
    std::filesystem::create_directories(fallback, ec);
    return fallback;
}

} // namespace dashcam::config

// ─── dashcam::camera (AttributeDictionary) ────────────────────────────────────

namespace dashcam::camera {

const AttributeEntry* AttributeDictionary::resolve(const std::string& alias,
                                                    const std::string& cameraType) const {
    auto toLower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return s;
    };
    const std::string needle = toLower(alias);
    for (const auto& e : entries) {
        if (e.type != "any" && e.type != cameraType) continue;
        for (const auto& a : e.aliases) {
            if (toLower(a) == needle) return &e;
        }
    }
    return nullptr;
}

bool AttributeDictionary::load(const std::string& filePath, AttributeDictionary& dict,
                                const dashcam::log::LogCallback& log) {
    dict.entries.clear();
    pugi::xml_document doc;
    if (!doc.load_file(filePath.c_str())) {
        // Reuse file-local doLog via the anonymous namespace helper.
        if (log) log(dashcam::log::LogLevel::ERROR,
                     ("cannot parse '" + filePath + "'").c_str());
        return false;
    }
    pugi::xml_node root = doc.child("CameraAttributeDictionary");
    if (!root) {
        if (log) log(dashcam::log::LogLevel::ERROR,
                     ("missing <CameraAttributeDictionary> root in '" + filePath + "'").c_str());
        return false;
    }
    for (auto node : root.children("Attribute")) {
        AttributeEntry e;
        e.gstProperty = node.attribute("gstProperty").as_string();
        e.type        = node.attribute("type").as_string("any");
        e.valueType   = parseValueType(node.attribute("valueType").as_string());
        for (auto alias : node.children("Alias"))
            e.aliases.push_back(alias.text().as_string());
        if (!e.gstProperty.empty())
            dict.entries.push_back(std::move(e));
    }
    return true;
}

} // namespace dashcam::camera
