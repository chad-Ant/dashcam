#include "libcamera.h"
#include "libconfig.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <pugixml.hpp>

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

    if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
        n.append_attribute("min").set_value(static_cast<double>(v.minValue()));
        n.append_attribute("max").set_value(static_cast<double>(v.maxValue()));
        n.append_attribute("step").set_value(static_cast<double>(v.step()));
        n.append_attribute("default").set_value(static_cast<double>(v.defaultValue()));
        n.text().set(static_cast<double>(static_cast<T>(v)));
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
}

static void parseCamera(pugi::xml_node node, CameraConfig& cam,
                        const dashcam::log::LogCallback& log) {
    cam.name = node.attribute("name").as_string(cam.name.c_str());
    cam.type = node.attribute("type").as_string(cam.type.c_str());

    readVar(node, cam.enabled,     log);
    readVar(node, cam.device,      log);
    readVar(node, cam.sensorId,    log);
    readVar(node, cam.formatIndex, log);

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
    readVar(node, sys.archivePath,  log);
    readVar(node, sys.warmupFrames, log);
}

static void parsePipeline(pugi::xml_node node, PipelineConfig& p,
                          const dashcam::log::LogCallback& log) {
    readVar(node, p.captureTimeoutMs,     log);
    readVar(node, p.stateChangeTimeoutMs, log);
    readVar(node, p.eosTimeoutMs,         log);
    readVar(node, p.captureQueueDepth,    log);
    readVar(node, p.appsinkMaxBuffers,    log);
}

static void parseRecording(pugi::xml_node node, RecordingConfig& r,
                           const dashcam::log::LogCallback& log) {
    readVar(node, r.recordFps,  log);
    readVar(node, r.queueDepth, log);
}

static void parseDetection(pugi::xml_node node, DetectionConfig& d,
                           const dashcam::log::LogCallback& log) {
    readVar(node, d.laneTargetHz,      log);
    readVar(node, d.laneReferenceY,    log);
    readVar(node, d.signTargetHz,      log);
    readVar(node, d.signConfThreshold, log);
    readVar(node, d.signNmsThreshold,  log);
    readVar(node, d.driverTargetHz,    log);
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
}

static void writeCamera(pugi::xml_node parent, const CameraConfig& cam) {
    pugi::xml_node n = parent.append_child("Camera");
    n.append_attribute("name").set_value(cam.name.c_str());
    n.append_attribute("type").set_value(cam.type.c_str());

    writeVar(n, cam.enabled);
    writeVar(n, cam.device);
    writeVar(n, cam.sensorId);
    writeVar(n, cam.formatIndex);

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
    writeVar(n, sys.archivePath);
    writeVar(n, sys.warmupFrames);
}

static void writePipeline(pugi::xml_node parent, const PipelineConfig& p) {
    pugi::xml_node n = parent.append_child("Pipeline");
    writeVar(n, p.captureTimeoutMs);
    writeVar(n, p.stateChangeTimeoutMs);
    writeVar(n, p.eosTimeoutMs);
    writeVar(n, p.captureQueueDepth);
    writeVar(n, p.appsinkMaxBuffers);
}

static void writeRecording(pugi::xml_node parent, const RecordingConfig& r) {
    pugi::xml_node n = parent.append_child("Recording");
    writeVar(n, r.recordFps);
    writeVar(n, r.queueDepth);
}

static void writeDetection(pugi::xml_node parent, const DetectionConfig& d) {
    pugi::xml_node n = parent.append_child("Detection");
    writeVar(n, d.laneTargetHz);
    writeVar(n, d.laneReferenceY);
    writeVar(n, d.signTargetHz);
    writeVar(n, d.signConfThreshold);
    writeVar(n, d.signNmsThreshold);
    writeVar(n, d.driverTargetHz);
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

    if (!doc.save_file(filePath.c_str(), "  ")) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "cannot write '%s'", filePath.c_str());
        return false;
    }
    return true;
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
