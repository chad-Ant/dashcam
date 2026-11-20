/**
 * @file SettingsStore.h
 * @brief Portable /settings.xml format for the user's Settings-tab values.
 *
 * The file is a small fixed-shape XML document, written by the firmware:
 *
 *     <?xml version="1.0" encoding="UTF-8"?>
 *     <settings version="1">
 *       <masterVolume>10</masterVolume>
 *       <brightness>100</brightness>
 *       <micThreshold>50</micThreshold>
 *     </settings>
 *
 * Every value is a percentage, 0..100. The reader is not a general XML parser:
 * it needs the <settings> root and reads each known element's integer. A
 * missing or invalid element keeps the caller's default for that value, so a
 * hand-edited file with one bad line still applies the rest; unknown elements
 * are ignored.
 */
#pragma once
#include "GenerationFiles.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct Es3Settings {
    uint8_t masterVolume;
    uint8_t brightness;
    uint8_t micThreshold;
};

namespace es3storage {
constexpr char kSettingsFile[] = "/settings.xml";
constexpr char kSettingsBackup[] = "/settings.xml.bak";
constexpr char kSettingsTemporary[] = "/settings.xml.tmp";
constexpr GenerationPaths kSettingsPaths{kSettingsFile, kSettingsBackup, kSettingsTemporary};
/** Largest file accepted (a formatted file is about 190 bytes). */
constexpr size_t kSettingsBytes = 512;

inline bool sameSettings(const Es3Settings& a, const Es3Settings& b) {
    return a.masterVolume == b.masterVolume && a.brightness == b.brightness &&
        a.micThreshold == b.micThreshold;
}

/** Finds needle in the first size bytes of text (text need not be terminated). */
inline const char* findText(const char* text, size_t size, const char* needle) {
    const size_t length = strlen(needle);
    for (size_t i = 0; length <= size && i <= size - length; ++i) {
        if (memcmp(text + i, needle, length) == 0) return text + i;
    }
    return nullptr;
}

/** Reads <name> 0..100 </name>; false leaves value unchanged. */
inline bool parsePercentElement(const char* text, size_t size, const char* name, uint8_t& value) {
    char open[32], close[33];
    if (snprintf(open, sizeof(open), "<%s>", name) >= int(sizeof(open)) ||
        snprintf(close, sizeof(close), "</%s>", name) >= int(sizeof(close))) return false;
    const char* p = findText(text, size, open);
    if (!p) return false;
    const char* end = text + size;
    p += strlen(open);
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
    int parsed = 0, digits = 0;
    while (p < end && *p >= '0' && *p <= '9') {
        parsed = parsed * 10 + (*p++ - '0');
        if (++digits > 3) return false;
    }
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
    const size_t closeLength = strlen(close);
    if (!digits || parsed > 100 || size_t(end - p) < closeLength ||
        memcmp(p, close, closeLength) != 0) return false;
    value = static_cast<uint8_t>(parsed);
    return true;
}

/**
 * Parses a settings document over settings, which holds the defaults on entry.
 * Returns false (settings unchanged) unless the text has a <settings> root
 * element; then each valid known element replaces its default.
 */
inline bool parseSettings(const char* text, size_t size, Es3Settings& settings) {
    if (!text || !size || size > kSettingsBytes ||
        (!findText(text, size, "<settings>") && !findText(text, size, "<settings ")) ||
        !findText(text, size, "</settings>")) return false;
    Es3Settings parsed = settings;
    (void)parsePercentElement(text, size, "masterVolume", parsed.masterVolume);
    (void)parsePercentElement(text, size, "brightness", parsed.brightness);
    (void)parsePercentElement(text, size, "micThreshold", parsed.micThreshold);
    settings = parsed;
    return true;
}

/** Formats settings (values clamped to 100); returns the byte count, 0 on error. */
inline size_t formatSettings(const Es3Settings& s, char (&out)[kSettingsBytes]) {
    const auto pct = [](uint8_t v) { return v > 100 ? 100u : unsigned(v); };
    const int size = snprintf(out, sizeof(out),
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<settings version=\"1\">\n"
        "  <masterVolume>%u</masterVolume>\n"
        "  <brightness>%u</brightness>\n"
        "  <micThreshold>%u</micThreshold>\n"
        "</settings>\n",
        pct(s.masterVolume), pct(s.brightness), pct(s.micThreshold));
    return size > 0 && size_t(size) < sizeof(out) ? size_t(size) : 0;
}

/** Loads the primary file, else the backup. False keeps the defaults. */
template<class Store>
bool loadSettings(Store& store, Es3Settings& settings) {
    return loadGeneration(store, kSettingsPaths, settings);
}
template<class Store>
bool saveSettings(Store& store, const Es3Settings& settings) {
    if (settings.masterVolume > 100 || settings.brightness > 100 || settings.micThreshold > 100) return false;
    return saveGeneration(store, kSettingsPaths, settings, sameSettings);
}
}  // namespace es3storage
