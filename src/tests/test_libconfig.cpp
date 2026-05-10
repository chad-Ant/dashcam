#include "libconfig.h"
#include <cassert>
#include <cstdio>
#include <filesystem>
#include <iostream>

using namespace dashcam::config;
namespace fs = std::filesystem;

// ── helpers ──────────────────────────────────────────────────────────────────

static bool eq(float a, float b) { return std::abs(a - b) < 1e-4f; }

static void check(bool cond, const char* label) {
    if (cond) {
        std::cout << "  PASS  " << label << "\n";
    } else {
        std::cerr << "  FAIL  " << label << "\n";
    }
}

// ── test 1: load the real dashcam.xml ────────────────────────────────────────

static bool test_load_real_file() {
    std::cout << "\n--- Test 1: load config/dashcam.xml ---\n";

    AppConfig cfg;
    bool ok = ConfigReader::load("config/dashcam.xml", cfg);
    check(ok, "load() returns true");

    // Encoder defaults (file has no overrides for these right now)
    check(cfg.encoder.bitrate == 8000,             "encoder.bitrate == 8000");
    check(cfg.encoder.speedPreset == "ultrafast",  "encoder.speedPreset == \"ultrafast\"");
    check(cfg.encoder.keyIntMax == 60,             "encoder.keyIntMax == 60");

    // Overlay defaults
    check(cfg.overlay.enabled == true,             "overlay.enabled == true");
    check(eq(cfg.overlay.backgroundOpacity, 0.5f), "overlay.backgroundOpacity == 0.5");
    check(eq(cfg.overlay.fontSize, 14.0f),         "overlay.fontSize == 14.0");
    check(cfg.overlay.fontFace == "Monospace Bold","overlay.fontFace == \"Monospace Bold\"");

    // Camera list — currently empty in the file
    check(cfg.cameras.empty(), "cameras list is empty (intentionally)");

    // System defaults
    check(cfg.system.archivePath == "./archive",   "system.archivePath == \"./archive\"");
    check(cfg.system.warmupFrames == 9,            "system.warmupFrames == 9");

    return ok;
}

// ── test 2: round-trip (save then load) ──────────────────────────────────────

static bool test_roundtrip() {
    std::cout << "\n--- Test 2: round-trip save → load ---\n";

    const std::string tmpFile = "/tmp/libconfig_roundtrip_test.xml";

    // Build a config with two cameras and non-default values.
    AppConfig src;
    src.encoder.bitrate     = 12000;
    src.encoder.speedPreset = "medium";
    src.encoder.keyIntMax   = 120;
    src.overlay.enabled     = false;
    src.overlay.fontSize    = 18.0f;
    src.system.archivePath  = "/mnt/ssd/clips";
    src.system.warmupFrames = 30;

    CameraConfig csi;
    csi.name        = "front";
    csi.type        = "CSI";
    csi.sensorId    = 0;
    csi.formatIndex = 4;
    csi.capabilities["exposuretimerange"] = "13000 13000";
    csi.capabilities["aelock"]            = "true";
    src.cameras.push_back(csi);

    CameraConfig usb;
    usb.name        = "cabin-left";
    usb.type        = "USB";
    usb.device      = "/dev/video2";
    usb.formatIndex = 0;
    src.cameras.push_back(usb);

    bool saved = ConfigReader::save(tmpFile, src);
    check(saved, "save() returns true");
    if (!saved) return false;

    // Load it back and verify every field.
    AppConfig dst;
    bool loaded = ConfigReader::load(tmpFile, dst);
    check(loaded, "load() round-trip returns true");

    check(dst.encoder.bitrate     == 12000,   "encoder.bitrate round-trip");
    check(dst.encoder.speedPreset == "medium", "encoder.speedPreset round-trip");
    check(dst.encoder.keyIntMax   == 120,     "encoder.keyIntMax round-trip");
    check(dst.overlay.enabled     == false,   "overlay.enabled round-trip");
    check(eq(dst.overlay.fontSize, 18.0f),    "overlay.fontSize round-trip");
    check(dst.system.archivePath  == "/mnt/ssd/clips", "system.archivePath round-trip");
    check(dst.system.warmupFrames == 30,      "system.warmupFrames round-trip");

    check(dst.cameras.size() == 2,            "cameras count == 2");
    if (dst.cameras.size() >= 2) {
        const auto& c0 = dst.cameras[0];
        check(c0.name        == "front",    "cam[0].name == \"front\"");
        check(c0.type        == "CSI",      "cam[0].type == \"CSI\"");
        check(c0.sensorId    == 0,          "cam[0].sensorId == 0");
        check(c0.formatIndex == 4,          "cam[0].formatIndex == 4");
        check(c0.capabilities.count("exposuretimerange") &&
              c0.capabilities.at("exposuretimerange") == "13000 13000",
              "cam[0] capability exposuretimerange");
        check(c0.capabilities.count("aelock") &&
              c0.capabilities.at("aelock") == "true",
              "cam[0] capability aelock");

        const auto& c1 = dst.cameras[1];
        check(c1.name        == "cabin-left",   "cam[1].name == \"cabin-left\"");
        check(c1.type        == "USB",          "cam[1].type == \"USB\"");
        check(c1.device      == "/dev/video2",  "cam[1].device == \"/dev/video2\"");
        check(c1.formatIndex == 0,              "cam[1].formatIndex == 0");
        check(c1.capabilities.empty(),          "cam[1] has no capabilities");
    }

    fs::remove(tmpFile);
    return loaded;
}

// ── test 3: missing file → graceful failure ───────────────────────────────────

static bool test_missing_file() {
    std::cout << "\n--- Test 3: missing file → graceful failure ---\n";

    AppConfig cfg;
    bool ok = ConfigReader::load("/nonexistent/path/no_such_file.xml", cfg);
    check(!ok, "load() returns false for missing file");
    // Defaults must still be intact.
    check(cfg.encoder.bitrate == 8000, "defaults preserved after failed load");

    return !ok;
}

// ── test 4: missing root node → graceful failure ─────────────────────────────

static bool test_bad_root() {
    std::cout << "\n--- Test 4: wrong root node → graceful failure ---\n";

    const std::string tmpFile = "/tmp/libconfig_badroot_test.xml";
    {
        std::FILE* f = std::fopen(tmpFile.c_str(), "w");
        if (!f) { std::cerr << "  SKIP  (cannot write temp file)\n"; return true; }
        std::fputs("<?xml version=\"1.0\"?><NotDashcamConfig/>", f);
        std::fclose(f);
    }

    AppConfig cfg;
    bool ok = ConfigReader::load(tmpFile, cfg);
    check(!ok, "load() returns false for wrong root node");
    check(cfg.encoder.bitrate == 8000, "defaults preserved after wrong-root load");

    fs::remove(tmpFile);
    return !ok;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    bool t1 = test_load_real_file();
    bool t2 = test_roundtrip();
    bool t3 = test_missing_file();
    bool t4 = test_bad_root();

    std::cout << "\n=== Results ===\n";
    std::cout << "Test 1 (load real file):    " << (t1 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 2 (round-trip):        " << (t2 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 3 (missing file):      " << (t3 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 4 (bad root node):     " << (t4 ? "PASS" : "FAIL") << "\n";

    return (t1 && t2 && t3 && t4) ? 0 : 1;
}
