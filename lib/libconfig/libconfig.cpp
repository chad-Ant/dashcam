#include "libcamera.h"
#include "libconfig.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <pugixml.hpp>

// ─── file-local log helper ────────────────────────────────────────────────────

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

} // namespace

// ─── file-local helpers ───────────────────────────────────────────────────────

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

static void parseEncoder(pugi::xml_node node, EncoderConfig& enc) {
    if (auto n = node.child("Bitrate"))     enc.bitrate     = n.text().as_int(enc.bitrate);
    if (auto n = node.child("SpeedPreset")) enc.speedPreset = n.text().as_string(enc.speedPreset.c_str());
    if (auto n = node.child("KeyIntMax"))   enc.keyIntMax   = n.text().as_int(enc.keyIntMax);
    if (auto n = node.child("Tune"))        enc.tune        = n.text().as_string(enc.tune.c_str());
}

static void parseOverlay(pugi::xml_node node, OverlayConfig& ovl) {
    if (auto n = node.child("Enabled"))           ovl.enabled           = n.text().as_bool(ovl.enabled);
    if (auto n = node.child("BackgroundOpacity")) ovl.backgroundOpacity = n.text().as_float(ovl.backgroundOpacity);
    if (auto n = node.child("FontSize"))          ovl.fontSize          = n.text().as_float(ovl.fontSize);
    if (auto n = node.child("FontFace"))          ovl.fontFace          = n.text().as_string(ovl.fontFace.c_str());
}

static void parseCamera(pugi::xml_node node, CameraConfig& cam) {
    cam.name = node.attribute("name").as_string(cam.name.c_str());
    cam.type = node.attribute("type").as_string(cam.type.c_str());
    if (auto n = node.child("Enabled"))     cam.enabled     = n.text().as_bool(cam.enabled);
    if (auto n = node.child("Device"))      cam.device      = n.text().as_string(cam.device.c_str());
    if (auto n = node.child("SensorId"))    cam.sensorId    = n.text().as_int(cam.sensorId);
    if (auto n = node.child("FormatIndex")) cam.formatIndex = n.text().as_int(cam.formatIndex);

    if (auto caps = node.child("Capabilities")) {
        for (auto cap : caps.children("Capability")) {
            const char* key = cap.attribute("name").value();
            const char* val = cap.attribute("value").value();
            if (key && *key)
                cam.capabilities[key] = val;
        }
    }
}

static void parseSystem(pugi::xml_node node, SystemConfig& sys) {
    if (auto n = node.child("ArchivePath"))  sys.archivePath  = n.text().as_string(sys.archivePath.c_str());
    if (auto n = node.child("WarmupFrames")) sys.warmupFrames = n.text().as_int(sys.warmupFrames);
}

static void writeEncoder(pugi::xml_node parent, const EncoderConfig& enc) {
    pugi::xml_node n = parent.append_child("Encoder");
    n.append_child("Bitrate").text().set(enc.bitrate);
    n.append_child("SpeedPreset").text().set(enc.speedPreset.c_str());
    n.append_child("KeyIntMax").text().set(enc.keyIntMax);
    n.append_child("Tune").text().set(enc.tune.c_str());
}

static void writeOverlay(pugi::xml_node parent, const OverlayConfig& ovl) {
    pugi::xml_node n = parent.append_child("Overlay");
    n.append_child("Enabled").text().set(ovl.enabled);
    n.append_child("BackgroundOpacity").text().set(static_cast<double>(ovl.backgroundOpacity));
    n.append_child("FontSize").text().set(static_cast<double>(ovl.fontSize));
    n.append_child("FontFace").text().set(ovl.fontFace.c_str());
}

static void writeCamera(pugi::xml_node parent, const CameraConfig& cam) {
    pugi::xml_node n = parent.append_child("Camera");
    n.append_attribute("name").set_value(cam.name.c_str());
    n.append_attribute("type").set_value(cam.type.c_str());
    n.append_child("Enabled").text().set(cam.enabled);
    n.append_child("Device").text().set(cam.device.c_str());
    n.append_child("SensorId").text().set(cam.sensorId);
    n.append_child("FormatIndex").text().set(cam.formatIndex);
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
    n.append_child("ArchivePath").text().set(sys.archivePath.c_str());
    n.append_child("WarmupFrames").text().set(sys.warmupFrames);
}

} // namespace

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

    if (auto enc  = root.child("Encoder")) parseEncoder(enc, config.encoder);
    if (auto ovl  = root.child("Overlay")) parseOverlay(ovl, config.overlay);
    if (auto sys  = root.child("System"))  parseSystem(sys,  config.system);

    if (auto cams = root.child("Cameras")) {
        config.cameras.clear();
        for (auto cam : cams.children("Camera")) {
            CameraConfig cc;
            parseCamera(cam, cc);
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
        doLog(log, dashcam::log::LogLevel::ERROR,
              "cannot parse '%s'", filePath.c_str());
        return false;
    }
    pugi::xml_node root = doc.child("CameraAttributeDictionary");
    if (!root) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "missing <CameraAttributeDictionary> root in '%s'", filePath.c_str());
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
