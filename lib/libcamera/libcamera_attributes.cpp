// AttributeDictionary (declared in libcamera.h): camera_attributes.xml → the
// GStreamer source-element properties Camera_GST writes.  Lives in libcamera
// with the class it implements; libconfig does not depend on libcamera.
#include "libcamera.h"

#include <algorithm>
#include <cctype>
#include <pugixml.hpp>

namespace dashcam::camera {

namespace {

AttributeValueType parseValueType(const char* str) {
    if (!str || !*str) return AttributeValueType::String;
    const std::string s(str);
    if (s == "int")            return AttributeValueType::Int;
    if (s == "float")          return AttributeValueType::Float;
    if (s == "bool")           return AttributeValueType::Bool;
    if (s == "bool_from_zero") return AttributeValueType::BoolFromZero;
    if (s == "range_string")   return AttributeValueType::RangeString;
    if (s == "v4l2_control")   return AttributeValueType::V4l2Control;
    return AttributeValueType::String;
}

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

} // namespace

const AttributeEntry* AttributeDictionary::resolve(const std::string& alias,
                                                    const std::string& cameraType) const {
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
        if (log) log(dashcam::log::LogLevel::ERROR, "cannot parse '" + filePath + "'");
        return false;
    }
    pugi::xml_node root = doc.child("CameraAttributeDictionary");
    if (!root) {
        if (log) log(dashcam::log::LogLevel::ERROR,
                     "missing <CameraAttributeDictionary> root in '" + filePath + "'");
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
