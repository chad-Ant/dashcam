#include "libcamera.h"   // AttributeDictionary (declared here, defined in libconfig.cpp)
#include "libconfig.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

using namespace dashcam::config;
namespace fs = std::filesystem;

// ── helpers ──────────────────────────────────────────────────────────────────

static bool eq(float a, float b) { return std::abs(a - b) < 1e-4f; }

static int g_fails = 0;  ///< Total failed checks; a test passes only if this stays put.

static void check(bool cond, const char* label) {
    if (cond) {
        std::cout << "  PASS  " << label << "\n";
    } else {
        std::cerr << "  FAIL  " << label << "\n";
        ++g_fails;   // propagate: an assertion failure must fail the test
    }
}

// ── test 1: load the real dashcam.xml ────────────────────────────────────────

static bool test_load_real_file() {
    std::cout << "\n--- Test 1: load config/dashcam.xml ---\n";
    const int before = g_fails;

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

    // Camera list — the file defines two cameras (csi0 CSI, usb0 USB).
    check(cfg.cameras.size() == 2, "cameras list has 2 entries");
    if (cfg.cameras.size() == 2) {
        check(cfg.cameras[0].name == "csi0" && cfg.cameras[0].type == "CSI", "cam[0] = csi0/CSI");
        check(cfg.cameras[1].name == "usb0" && cfg.cameras[1].type == "USB", "cam[1] = usb0/USB");
        check(cfg.cameras[0].device == "/dev/video0", "cam[0].device == \"/dev/video0\"");
        check(!cfg.cameras[0].attributeInfo.empty(),  "cam[0] has discovered attributeInfo");
    }

    // System defaults
    check(cfg.system.footagePath == "/user/output/footage", "system.footagePath == \"/user/output/footage\"");
    check(cfg.system.warmupFrames == 9,            "system.warmupFrames == 9");

    // Sections/fields absent from the on-disk file keep their built-in defaults.
    check(cfg.log.queueSize == 8192,               "log.queueSize default 8192 (section may be absent)");
    check(cfg.log.level == "debug",                "log.level default \"debug\"");
    check(cfg.log.flushOn == "warn",               "log.flushOn default \"warn\"");
    check(cfg.pipeline.branchQueueDepth == 2,      "pipeline.branchQueueDepth default 2");

    return g_fails == before;
}

// ── test 2: round-trip (save then load) ──────────────────────────────────────

static bool test_roundtrip() {
    std::cout << "\n--- Test 2: round-trip save → load ---\n";
    const int before = g_fails;

    const std::string tmpFile = "/tmp/libconfig_roundtrip_test.xml";

    // Build a config with two cameras and non-default values.
    AppConfig src;
    src.encoder.bitrate     = 12000;
    src.encoder.speedPreset = "medium";
    src.encoder.keyIntMax   = 120;
    src.overlay.enabled     = false;
    src.overlay.fontSize    = 18.0f;
    src.overlay.labelPadX   = 20.0f;
    src.overlay.labelPadY   = 16.0f;
    src.overlay.subtitleRateHz = 2.0f;
    src.overlay.staleTimeoutMs = 1500;
    src.recording.recordWidth  = 1280;
    src.recording.recordHeight = 720;
    src.system.footagePath  = "/mnt/ssd/footage";
    src.system.warmupFrames = 30;
    src.pipeline.branchQueueDepth = 5;
    src.log.queueSize     = 4096;
    src.log.rotateSizeKb  = 1024;
    src.log.rotateFiles   = 5;
    src.log.flushEverySec = 3;
    src.log.level         = "info";
    src.log.flushOn       = "error";
    src.detection.laneEnginePath   = "models/custom_lane.engine";
    src.detection.laneTargetHz     = 15;
    src.detection.laneBranchMaxFps = 30;
    src.detection.laneInputCropTop = 0.40f;
    src.detection.laneInputCropBottom = 0.10f;
    src.detection.driverEnginePath      = "models/custom_drowsy.engine";
    src.detection.driverTargetHz        = 4;
    src.detection.driverBranchMaxFps    = 5;
    src.detection.driverDrowsyThreshold = 0.65f;
    src.detection.driverFaceDetection   = false;
    src.detection.driverFaceModelPath   = "models/custom_yunet.onnx";
    src.detection.driverFaceScore       = 0.75f;
    src.detection.driverFaceDetectScale = 0.5f;

    src.driverScore.scoreInitial       = 90.0f;
    src.driverScore.scoreUpper         = 95.0f;
    src.driverScore.scoreLower         = -20.0f;
    src.driverScore.drowsyChunkSec     = 15.0f;
    src.driverScore.drowsyChunkPenalty = 12.0f;
    src.driverScore.awakeChunkSec      = 20.0f;
    src.driverScore.awakeChunkReward   = 4.0f;
    src.driverScore.laneDepartThresh   = 0.70f;
    src.driverScore.laneReturnSec      = 12.0f;
    src.driverScore.laneDriftPenalty   = 8.0f;
    src.driverScore.capDecayPerHour    = 5.0f;
    src.driverScore.capDecayFloor      = 40.0f;
    src.driverScore.cautionScore       = 55.0f;
    src.driverScore.warningScore       = 25.0f;
    src.driverScore.fatigueScore       = -5.0f;
    src.driverScore.fatigueSustainSec  = 120.0f;
    src.driverScore.acuteAlertSec      = 3.0f;
    src.driverScore.noFaceFreezes      = false;

    src.network.wifiConnectEnabled  = false;
    src.network.wifiSsid            = "MyDashcamNet";
    src.network.wifiTimeoutSec      = 30;
    src.network.wifiRequireInternet = true;
    src.network.timeSyncEnabled     = false;
    src.network.ntpServer           = "time.cloudflare.com";
    src.network.ntpPort             = 1230;
    src.network.ntpTimeoutMs        = 5000;
    src.network.ntpRetries          = 4;
    src.network.streamEnabled       = true;
    src.network.streamPort          = 9000;
    src.network.streamMaxClients    = 8;
    src.network.rtpEnabled          = true;
    src.network.rtpHost             = "192.168.1.42";
    src.network.rtpPort             = 5602;
    src.network.rtpBitrateKbps      = 6000;

    CameraConfig csi;
    csi.name        = "front";
    csi.type        = "CSI";
    csi.sensorId    = 0;
    csi.formatIndex = 4;
    csi.outWidth    = 640;
    csi.outHeight   = 480;
    csi.outFps      = 20.0f;
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
    check(eq(dst.overlay.labelPadX, 20.0f),   "overlay.labelPadX round-trip");
    check(eq(dst.overlay.labelPadY, 16.0f),   "overlay.labelPadY round-trip");
    check(eq(dst.overlay.subtitleRateHz, 2.0f), "overlay.subtitleRateHz round-trip");
    check(dst.overlay.staleTimeoutMs == 1500,   "overlay.staleTimeoutMs round-trip");
    check(dst.recording.recordWidth  == 1280, "recording.recordWidth round-trip");
    check(dst.recording.recordHeight == 720,  "recording.recordHeight round-trip");
    check(dst.system.footagePath  == "/mnt/ssd/footage", "system.footagePath round-trip");
    check(dst.system.warmupFrames == 30,      "system.warmupFrames round-trip");
    check(dst.pipeline.branchQueueDepth == 5, "pipeline.branchQueueDepth round-trip");
    check(dst.log.queueSize     == 4096,      "log.queueSize round-trip");
    check(dst.log.rotateSizeKb  == 1024,      "log.rotateSizeKb round-trip");
    check(dst.log.rotateFiles   == 5,         "log.rotateFiles round-trip");
    check(dst.log.flushEverySec == 3,         "log.flushEverySec round-trip");
    check(dst.log.level         == "info",    "log.level round-trip");
    check(dst.log.flushOn       == "error",   "log.flushOn round-trip");
    check(dst.detection.laneEnginePath == "models/custom_lane.engine",
                                              "detection.laneEnginePath round-trip");
    check(dst.detection.laneTargetHz     == 15, "detection.laneTargetHz round-trip");
    check(dst.detection.laneBranchMaxFps == 30, "detection.laneBranchMaxFps round-trip");
    check(eq(dst.detection.laneInputCropTop, 0.40f),
                                              "detection.laneInputCropTop round-trip");
    check(eq(dst.detection.laneInputCropBottom, 0.10f),
                                              "detection.laneInputCropBottom round-trip");
    check(dst.detection.driverEnginePath == "models/custom_drowsy.engine",
                                              "detection.driverEnginePath round-trip");
    check(dst.detection.driverTargetHz     == 4, "detection.driverTargetHz round-trip");
    check(dst.detection.driverBranchMaxFps == 5, "detection.driverBranchMaxFps round-trip");
    check(eq(dst.detection.driverDrowsyThreshold, 0.65f),
                                              "detection.driverDrowsyThreshold round-trip");
    check(dst.detection.driverFaceDetection == false,
                                              "detection.driverFaceDetection round-trip");
    check(dst.detection.driverFaceModelPath == "models/custom_yunet.onnx",
                                              "detection.driverFaceModelPath round-trip");
    check(eq(dst.detection.driverFaceScore, 0.75f),
                                              "detection.driverFaceScore round-trip");
    check(eq(dst.detection.driverFaceDetectScale, 0.5f),
                                              "detection.driverFaceDetectScale round-trip");

    check(eq(dst.driverScore.scoreInitial, 90.0f),       "driverScore.scoreInitial round-trip");
    check(eq(dst.driverScore.scoreUpper, 95.0f),         "driverScore.scoreUpper round-trip");
    check(eq(dst.driverScore.scoreLower, -20.0f),        "driverScore.scoreLower round-trip");
    check(eq(dst.driverScore.drowsyChunkSec, 15.0f),     "driverScore.drowsyChunkSec round-trip");
    check(eq(dst.driverScore.drowsyChunkPenalty, 12.0f), "driverScore.drowsyChunkPenalty round-trip");
    check(eq(dst.driverScore.awakeChunkSec, 20.0f),      "driverScore.awakeChunkSec round-trip");
    check(eq(dst.driverScore.awakeChunkReward, 4.0f),    "driverScore.awakeChunkReward round-trip");
    check(eq(dst.driverScore.laneDepartThresh, 0.70f),   "driverScore.laneDepartThresh round-trip");
    check(eq(dst.driverScore.laneReturnSec, 12.0f),      "driverScore.laneReturnSec round-trip");
    check(eq(dst.driverScore.laneDriftPenalty, 8.0f),    "driverScore.laneDriftPenalty round-trip");
    check(eq(dst.driverScore.capDecayPerHour, 5.0f),     "driverScore.capDecayPerHour round-trip");
    check(eq(dst.driverScore.capDecayFloor, 40.0f),      "driverScore.capDecayFloor round-trip");
    check(eq(dst.driverScore.cautionScore, 55.0f),       "driverScore.cautionScore round-trip");
    check(eq(dst.driverScore.warningScore, 25.0f),       "driverScore.warningScore round-trip");
    check(eq(dst.driverScore.fatigueScore, -5.0f),       "driverScore.fatigueScore round-trip");
    check(eq(dst.driverScore.fatigueSustainSec, 120.0f), "driverScore.fatigueSustainSec round-trip");
    check(eq(dst.driverScore.acuteAlertSec, 3.0f),       "driverScore.acuteAlertSec round-trip");
    check(dst.driverScore.noFaceFreezes == false,        "driverScore.noFaceFreezes round-trip");

    check(dst.network.wifiConnectEnabled == false,       "network.wifiConnectEnabled round-trip");
    check(dst.network.wifiSsid == "MyDashcamNet",        "network.wifiSsid round-trip");
    check(dst.network.wifiTimeoutSec == 30,              "network.wifiTimeoutSec round-trip");
    check(dst.network.wifiRequireInternet == true,       "network.wifiRequireInternet round-trip");
    check(dst.network.timeSyncEnabled == false,          "network.timeSyncEnabled round-trip");
    check(dst.network.ntpServer == "time.cloudflare.com","network.ntpServer round-trip");
    check(dst.network.ntpPort == 1230,                   "network.ntpPort round-trip");
    check(dst.network.ntpTimeoutMs == 5000,              "network.ntpTimeoutMs round-trip");
    check(dst.network.ntpRetries == 4,                   "network.ntpRetries round-trip");
    check(dst.network.streamEnabled == true,             "network.streamEnabled round-trip");
    check(dst.network.streamPort == 9000,                "network.streamPort round-trip");
    check(dst.network.streamMaxClients == 8,             "network.streamMaxClients round-trip");
    check(dst.network.rtpEnabled == true,                "network.rtpEnabled round-trip");
    check(dst.network.rtpHost == "192.168.1.42",         "network.rtpHost round-trip");
    check(dst.network.rtpPort == 5602,                   "network.rtpPort round-trip");
    check(dst.network.rtpBitrateKbps == 6000,            "network.rtpBitrateKbps round-trip");
    {
        std::ifstream saved(tmpFile);
        const std::string xml((std::istreambuf_iterator<char>(saved)),
                              std::istreambuf_iterator<char>());
        check(xml.find("NtpStepClock") == std::string::npos &&
              xml.find("NtpStepThresholdSec") == std::string::npos,
              "obsolete SNTP clock-step controls are not serialized");
    }

    check(dst.cameras.size() == 2,            "cameras count == 2");
    if (dst.cameras.size() >= 2) {
        const auto& c0 = dst.cameras[0];
        check(c0.name        == "front",    "cam[0].name == \"front\"");
        check(c0.type        == "CSI",      "cam[0].type == \"CSI\"");
        check(c0.sensorId    == 0,          "cam[0].sensorId == 0");
        check(c0.formatIndex == 4,          "cam[0].formatIndex == 4");
        check(c0.outWidth    == 640,        "cam[0].outWidth round-trip");
        check(c0.outHeight   == 480,        "cam[0].outHeight round-trip");
        check(eq(c0.outFps, 20.0f),         "cam[0].outFps round-trip");
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
    return g_fails == before;
}

// ── test 3: missing file → graceful failure ───────────────────────────────────

static bool test_missing_file() {
    std::cout << "\n--- Test 3: missing file → graceful failure ---\n";
    const int before = g_fails;

    AppConfig cfg;
    bool ok = ConfigReader::load("/nonexistent/path/no_such_file.xml", cfg);
    check(!ok, "load() returns false for missing file");
    // Defaults must still be intact.
    check(cfg.encoder.bitrate == 8000, "defaults preserved after failed load");

    return g_fails == before;
}

// ── test 4: missing root node → graceful failure ─────────────────────────────

static bool test_bad_root() {
    std::cout << "\n--- Test 4: wrong root node → graceful failure ---\n";

    const int before = g_fails;
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
    return g_fails == before;
}

// ── test 5: out-of-range values are clamped and a WARN is logged ─────────────

static bool test_clamping() {
    std::cout << "\n--- Test 5: value clamping + WARN ---\n";
    const int before = g_fails;

    const std::string tmpFile = "/tmp/libconfig_clamp_test.xml";
    {
        std::ofstream f(tmpFile);
        if (!f) { std::cerr << "  SKIP  (cannot write temp file)\n"; return true; }
        f << "<?xml version=\"1.0\"?><DashcamConfig>"
             "<Encoder><Bitrate>999999</Bitrate><KeyIntMax>0</KeyIntMax></Encoder>"
             "<Overlay><BackgroundOpacity>5.0</BackgroundOpacity><FontSize>200</FontSize></Overlay>"
             "<System><WarmupFrames>-5</WarmupFrames></System>"
             "<Log><QueueSize>1</QueueSize></Log></DashcamConfig>";
    }

    std::vector<std::string> warns;
    dashcam::log::LogCallback log = [&](dashcam::log::LogLevel lvl, const std::string& m) {
        if (lvl == dashcam::log::LogLevel::WARN) warns.push_back(m);
    };

    AppConfig cfg;
    check(ConfigReader::load(tmpFile, cfg, log), "load clamp file");
    check(cfg.encoder.bitrate     == 50000, "Bitrate 999999 -> clamped to max 50000");
    check(cfg.encoder.keyIntMax   == 1,     "KeyIntMax 0 -> clamped to min 1");
    check(eq(cfg.overlay.backgroundOpacity, 1.0f), "BackgroundOpacity 5.0 -> clamped to 1.0");
    check(eq(cfg.overlay.fontSize, 72.0f),         "FontSize 200 -> clamped to max 72");
    check(cfg.system.warmupFrames == 0,     "WarmupFrames -5 -> clamped to min 0");
    check(cfg.log.queueSize       == 256,   "Log QueueSize 1 -> clamped to min 256");
    check(warns.size() >= 6,                "a WARN was logged for each clamp (>=6)");

    fs::remove(tmpFile);
    return g_fails == before;
}

// ── test 6: saved XML uses clean ints and short-precision floats ──────────────

static bool test_saved_xml_formatting() {
    std::cout << "\n--- Test 6: saved XML formatting ---\n";
    const int before = g_fails;

    const std::string tmpFile = "/tmp/libconfig_fmt_test.xml";
    AppConfig def;  // built-in defaults (backgroundOpacity 0.85, step 0.05, …)
    check(ConfigReader::save(tmpFile, def), "save() defaults");

    std::ifstream in(tmpFile);
    const std::string xml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    auto has = [&](const char* s) { return xml.find(s) != std::string::npos; };

    check(has("default=\"0.85\""), "float default written cleanly as 0.85");
    check(has("step=\"0.05\""),    "float step written cleanly as 0.05");
    check(has("<Bitrate type=\"int\""), "int field tagged type=int");
    // The old bug wrote floats at ~17 sig-figs (0.85000002384185791); guard against it.
    check(!has("0.850000") && !has("0.050000") && !has("0000000"),
          "no long-precision float garbage in output");

    fs::remove(tmpFile);
    return g_fails == before;
}

// ── test 7: AttributeDictionary load + case/type-aware resolve ────────────────

static bool test_attribute_dictionary() {
    std::cout << "\n--- Test 7: AttributeDictionary load + resolve ---\n";
    const int before = g_fails;
    using namespace dashcam::camera;

    AttributeDictionary dict;
    check(AttributeDictionary::load("config/camera_attributes.xml", dict), "dict loads");
    check(!dict.entries.empty(), "dict has entries");

    // Alias collisions are split by camera type.
    const AttributeEntry* csiExp = dict.resolve("exposure", "CSI");
    const AttributeEntry* usbExp = dict.resolve("exposure", "USB");
    check(csiExp && csiExp->gstProperty == "exposuretimerange", "exposure/CSI -> exposuretimerange");
    check(usbExp && usbExp->gstProperty == "exposure",          "exposure/USB -> exposure");
    const AttributeEntry* csiSat = dict.resolve("saturation", "CSI");
    const AttributeEntry* usbSat = dict.resolve("saturation", "USB");
    check(csiSat && csiSat->valueType == AttributeValueType::Float, "saturation/CSI -> float");
    check(usbSat && usbSat->valueType == AttributeValueType::Int,   "saturation/USB -> int");

    check(dict.resolve("EXPOSURE", "CSI") == csiExp,               "resolve is case-insensitive");
    check(dict.resolve("Exposure Time, Absolute", "CSI") == csiExp, "multi-word alias with spaces/commas");
    check(dict.resolve("no_such_attribute", "CSI") == nullptr,     "unknown alias -> nullptr");
    check(dict.resolve("exposure", "GIGE") == nullptr,             "unmatched camera type -> nullptr");

    AttributeDictionary bad;
    check(!AttributeDictionary::load("/no/such/dict.xml", bad),    "missing dict file -> false");

    return g_fails == before;
}

// ── test 8: loadOrCreate() seeds a default config; configDir() is exe-relative ─

static bool test_load_or_create() {
    std::cout << "\n--- Test 8: loadOrCreate seeds default config ---\n";
    const int before = g_fails;

    const std::string root = "/tmp/libconfig_loadcreate";
    const std::string file = root + "/config/dashcam.xml";  // nested dir must be created
    fs::remove_all(root);
    check(!fs::exists(file), "config file absent to start");

    AppConfig cfg;
    check(ConfigReader::loadOrCreate(file, cfg), "loadOrCreate() returns true (creates)");
    check(fs::exists(file), "config file (and parent dir) created on disk");
    check(cfg.encoder.bitrate == 8000,            "created config has default bitrate 8000");
    check(cfg.encoder.speedPreset == "ultrafast", "created config has default speedPreset");
    check(eq(cfg.overlay.backgroundOpacity, 0.85f),"created config has default opacity 0.85");
    check(cfg.cameras.empty(),                    "created config has no cameras (pure defaults)");

    // A second call finds the file and loads it (no re-create).
    AppConfig cfg2;
    check(ConfigReader::loadOrCreate(file, cfg2),  "loadOrCreate() returns true when file exists");
    check(cfg2.encoder.bitrate == 8000,            "reloaded config matches");

    // configDir() resolves relative to the executable, ending in .../config.
    const std::string cdir = configDir();
    std::cout << "  configDir() -> " << cdir << "\n";
    check(cdir.size() >= 6 && cdir.substr(cdir.size() - 6) == "config", "configDir() ends in .../config");

    fs::remove_all(root);
    return g_fails == before;
}

// ── test 9: resolveStorageDir() — mount available vs. unavailable (SD removed) ─

static bool test_resolve_storage_dir() {
    std::cout << "\n--- Test 9: resolveStorageDir build-local fallback ---\n";
    const int before = g_fails;

    // Available: a writable path is returned as-is (and created).
    const std::string good = "/tmp/libconfig_avail_test";
    fs::remove_all(good);
    const std::string gotGood = resolveStorageDir(good, "unused_fallback");
    check(gotGood == good,      "writable preferred dir returned unchanged");
    check(fs::exists(gotGood),  "preferred dir created by resolveStorageDir");

    // Unavailable: parent is a regular file, so the dir can neither be created nor
    // written (ENOTDIR) — stands in for a removed SD card / stale mount.  The
    // build-local fallback exeRelativeDir("<name>") is returned and created.
    const std::string blocker = "/tmp/libconfig_blocker_file";
    fs::remove_all(blocker);
    { FILE* f = std::fopen(blocker.c_str(), "w"); if (f) std::fclose(f); }
    const std::string unavailable = blocker + "/cannot";  // parent is a file
    const std::string gotBad = resolveStorageDir(unavailable, "output_fallback_test");
    check(gotBad == exeRelativeDir("output_fallback_test"), "unavailable dir falls back to build-local");
    check(fs::exists(gotBad),                               "fallback dir was created");

    fs::remove_all(good);
    fs::remove(blocker);
    fs::remove_all(gotBad);
    return g_fails == before;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    bool t1 = test_load_real_file();
    bool t2 = test_roundtrip();
    bool t3 = test_missing_file();
    bool t4 = test_bad_root();
    bool t5 = test_clamping();
    bool t6 = test_saved_xml_formatting();
    bool t7 = test_attribute_dictionary();
    bool t8 = test_load_or_create();
    bool t9 = test_resolve_storage_dir();

    std::cout << "\n=== Results ===\n";
    std::cout << "Test 1 (load real file):    " << (t1 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 2 (round-trip):        " << (t2 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 3 (missing file):      " << (t3 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 4 (bad root node):     " << (t4 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 5 (clamping):          " << (t5 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 6 (xml formatting):    " << (t6 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 7 (attribute dict):    " << (t7 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 8 (loadOrCreate):      " << (t8 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 9 (storage fallback):  " << (t9 ? "PASS" : "FAIL") << "\n";

    return (t1 && t2 && t3 && t4 && t5 && t6 && t7 && t8 && t9) ? 0 : 1;
}
