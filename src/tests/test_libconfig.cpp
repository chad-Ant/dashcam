#include "libcamera.h"   // AttributeDictionary (libcamera_attributes.cpp)
#include "libconfig.h"
// es3video:: protocol ceilings, for the DisplayPreview range drift check (test 11).
#include "../../peripherals/es3c28p/lib/es3UsbLink/Es3VideoProtocol.h"
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

    std::vector<std::string> warns;
    dashcam::log::LogCallback log = [&](dashcam::log::LogLevel lvl, const std::string& m) {
        if (lvl == dashcam::log::LogLevel::WARN) warns.push_back(m);
    };
    AppConfig cfg;
    bool ok = ConfigReader::load("config/dashcam.xml", cfg, log);
    check(ok, "load() returns true");
    bool obsolete = false;
    for (const auto& w : warns) obsolete = obsolete || w.find("obsolete") != std::string::npos;
    check(!obsolete, "the shipped file carries no obsolete settings");

    // Recording defaults (the file has no <Recording> section)
    check(cfg.recording.segmentSec == 180,         "recording.segmentSec == 180");
    check(cfg.recording.syncIntervalMs == 1000,    "recording.syncIntervalMs == 1000");

    // Overlay defaults
    check(cfg.overlay.enabled == true,             "overlay.enabled == true");
    check(eq(cfg.overlay.backgroundOpacity, 0.5f), "overlay.backgroundOpacity == 0.5");
    check(eq(cfg.overlay.fontSize, 14.0f),         "overlay.fontSize == 14.0");
    check(cfg.overlay.fontFace == "Monospace Bold","overlay.fontFace == \"Monospace Bold\"");

    // Camera list — the file defines two cameras (csi0 CSI, and the pinned
    // v0.4 dashcam camera: a stable by-id link, FormatIndex 4, a comment inside).
    check(cfg.cameras.size() == 2, "cameras list has 2 entries");
    if (cfg.cameras.size() == 2) {
        check(cfg.cameras[0].name == "csi0" && cfg.cameras[0].type == "CSI", "cam[0] = csi0/CSI");
        check(cfg.cameras[1].name == "dashcam" && cfg.cameras[1].type == "USB", "cam[1] = dashcam/USB");
        check(cfg.cameras[0].device == "/dev/video0", "cam[0].device == \"/dev/video0\"");
        check(!cfg.cameras[0].attributeInfo.empty(),  "cam[0] has discovered attributeInfo");
        check((std::string)cfg.cameras[1].device ==
                  "/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0",
              "cam[1].device == the UGREEN's by-id link");
        check(cfg.cameras[1].formatIndex == 4 && (bool)cfg.cameras[1].enabled,
              "cam[1] enabled, FormatIndex 4 (1080p30)");
    }

    // System defaults
    check(cfg.system.footagePath == "/user/output/footage", "system.footagePath == \"/user/output/footage\"");

    // Sections/fields absent from the on-disk file keep their built-in defaults.
    check(cfg.log.queueSize == 8192,               "log.queueSize default 8192 (section may be absent)");
    check(cfg.log.level == "debug",                "log.level default \"debug\"");
    check(cfg.log.flushOn == "warn",               "log.flushOn default \"warn\"");
    check(cfg.log.telemetryCsv == false,           "log.telemetryCsv default false");
    check(cfg.overlay.vehicleDetail == true,       "overlay.vehicleDetail default true");
    check(cfg.pipeline.branchQueueDepth == 2,      "pipeline.branchQueueDepth default 2");
    check(!cfg.displayPreview.enabled,             "displayPreview.enabled false (preview off in the shipped file)");

    return g_fails == before;
}

// ── test 2: round-trip (save then load) ──────────────────────────────────────

static bool test_roundtrip() {
    std::cout << "\n--- Test 2: round-trip save → load ---\n";
    const int before = g_fails;

    const std::string tmpFile = "/tmp/libconfig_roundtrip_test.xml";

    // Build a config with two cameras and non-default values.
    AppConfig src;
    src.overlay.enabled     = false;
    src.overlay.fontSize    = 18.0f;
    src.overlay.labelPadX   = 20.0f;
    src.overlay.labelPadY   = 16.0f;
    src.overlay.subtitleRateHz = 2.0f;
    src.overlay.staleTimeoutMs = 1500;
    src.overlay.vehicleDetail  = false;
    src.recording.segmentSec          = 60;
    src.recording.maxFootageGB        = 123.0f;
    src.recording.minFreeGB           = 7.0f;
    src.recording.stallTimeoutMs      = 2500;
    src.recording.firstFrameTimeoutMs = 9000;
    src.recording.retryIntervalSec    = 5;
    src.recording.syncIntervalMs      = 2500;
    src.recording.exposureMode        = std::string("camera");
    src.recording.targetLuma          = 90;
    src.recording.nightLuma           = 50;
    src.recording.focusMode           = std::string("fixed");
    src.recording.focusAbsolute       = 250;
    src.system.footagePath  = "/mnt/ssd/footage";
    src.pipeline.branchQueueDepth = 5;
    src.log.queueSize     = 4096;
    src.log.rotateSizeKb  = 1024;
    src.log.rotateFiles   = 5;
    src.log.flushEverySec = 3;
    src.log.level         = "info";
    src.log.flushOn       = "error";
    src.log.telemetryCsv  = true;
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
    src.network.clockSetEnabled     = false;
    src.network.gpsTimeFallback     = false;
    src.network.streamEnabled       = true;
    src.network.streamPort          = 9000;
    src.network.streamMaxClients    = 8;
    src.network.streamBindAddress   = "10.0.0.5";
    src.network.rtpEnabled          = true;
    src.network.rtpHost             = "192.168.1.42";
    src.network.rtpPort             = 5602;
    src.network.rtpBitrateKbps      = 6000;
    src.network.controlEnabled      = true;
    src.network.controlPort         = 8123;
    src.network.controlMaxClients   = 5;
    src.network.controlBindAddress  = "127.0.0.1";
    src.network.controlAllowlist    = "127.0.0.1,192.168.1.10";
    src.network.controlAuthToken    = "s3cr3t-psk";

    src.displayPreview.enabled            = true;
    src.displayPreview.device             = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_12:34:56:78:9A:BC-if00";
    src.displayPreview.targetFps          = 15;
    // Unsupported (validateDisplayPreview rejects it), but the file must keep
    // exactly what the operator wrote: a round trip of the default proves nothing.
    src.displayPreview.fit                = "stretch";
    src.displayPreview.jpegQuality        = 80;
    src.displayPreview.minimumJpegQuality = 30;
    src.displayPreview.maxJpegBytes       = 32768;
    src.displayPreview.maxSourceJpegBytes = 2097152;
    src.displayPreview.chunkDataBytes     = 2048;
    src.displayPreview.maxFramesInFlight  = 2;
    src.displayPreview.assemblyTimeoutMs  = 750;
    src.displayPreview.staleAfterMs       = 800;
    src.displayPreview.placeholderAfterMs = 3000;
    src.displayPreview.heartbeatMs        = 1500;
    src.displayPreview.reconnectMs        = 5000;

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
    // A control range past float's 2^24 integer limit (V4L2 ranges are int32): it must come back exact.
    CameraAttributeInfo wide;
    wide.name     = "wide_range";
    wide.writable = true;
    wide.readable = true;
    wide.minValue = -2147483648.0;
    wide.maxValue = 2147483647.0;
    wide.step     = 16777217.0;
    usb.attributeInfo.push_back(wide);
    src.cameras.push_back(usb);

    bool saved = ConfigReader::save(tmpFile, src);
    check(saved, "save() returns true");
    if (!saved) return false;

    // Load it back and verify every field.
    AppConfig dst;
    bool loaded = ConfigReader::load(tmpFile, dst);
    check(loaded, "load() round-trip returns true");

    check(dst.overlay.enabled     == false,   "overlay.enabled round-trip");
    check(eq(dst.overlay.fontSize, 18.0f),    "overlay.fontSize round-trip");
    check(eq(dst.overlay.labelPadX, 20.0f),   "overlay.labelPadX round-trip");
    check(eq(dst.overlay.labelPadY, 16.0f),   "overlay.labelPadY round-trip");
    check(eq(dst.overlay.subtitleRateHz, 2.0f), "overlay.subtitleRateHz round-trip");
    check(dst.overlay.staleTimeoutMs == 1500,   "overlay.staleTimeoutMs round-trip");
    check(dst.overlay.vehicleDetail == false,   "overlay.vehicleDetail round-trip");
    check(dst.recording.segmentSec          == 60,     "recording.segmentSec round-trip");
    check(dst.recording.maxFootageGB        == 123.0f, "recording.maxFootageGB round-trip");
    check(dst.recording.minFreeGB           == 7.0f,   "recording.minFreeGB round-trip");
    check(dst.recording.stallTimeoutMs      == 2500,   "recording.stallTimeoutMs round-trip");
    check(dst.recording.firstFrameTimeoutMs == 9000,   "recording.firstFrameTimeoutMs round-trip");
    check(dst.recording.retryIntervalSec    == 5,      "recording.retryIntervalSec round-trip");
    check(dst.recording.syncIntervalMs      == 2500,   "recording.syncIntervalMs round-trip");
    check((std::string)dst.recording.exposureMode == "camera", "recording.exposureMode round-trip");
    check(dst.recording.targetLuma          == 90,     "recording.targetLuma round-trip");
    check(dst.recording.nightLuma           == 50,     "recording.nightLuma round-trip");
    check((std::string)dst.recording.focusMode == "fixed", "recording.focusMode round-trip");
    check(dst.recording.focusAbsolute       == 250,    "recording.focusAbsolute round-trip");
    check(dst.system.footagePath  == "/mnt/ssd/footage", "system.footagePath round-trip");
    check(dst.pipeline.branchQueueDepth == 5, "pipeline.branchQueueDepth round-trip");
    check(dst.log.queueSize     == 4096,      "log.queueSize round-trip");
    check(dst.log.rotateSizeKb  == 1024,      "log.rotateSizeKb round-trip");
    check(dst.log.rotateFiles   == 5,         "log.rotateFiles round-trip");
    check(dst.log.flushEverySec == 3,         "log.flushEverySec round-trip");
    check(dst.log.level         == "info",    "log.level round-trip");
    check(dst.log.flushOn       == "error",   "log.flushOn round-trip");
    check(dst.log.telemetryCsv  == true,      "log.telemetryCsv round-trip");
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
    check(!dst.network.clockSetEnabled,                  "network.clockSetEnabled round-trip");
    check(!dst.network.gpsTimeFallback,                  "network.gpsTimeFallback round-trip");
    check(dst.network.streamEnabled == true,             "network.streamEnabled round-trip");
    check(dst.network.streamPort == 9000,                "network.streamPort round-trip");
    check(dst.network.streamMaxClients == 8,             "network.streamMaxClients round-trip");
    check(dst.network.streamBindAddress == "10.0.0.5",   "network.streamBindAddress round-trip");
    check(dst.network.rtpEnabled == true,                "network.rtpEnabled round-trip");
    check(dst.network.rtpHost == "192.168.1.42",         "network.rtpHost round-trip");
    check(dst.network.rtpPort == 5602,                   "network.rtpPort round-trip");
    check(dst.network.rtpBitrateKbps == 6000,            "network.rtpBitrateKbps round-trip");
    check(dst.network.controlEnabled == true,            "network.controlEnabled round-trip");
    check(dst.network.controlPort == 8123,               "network.controlPort round-trip");
    check(dst.network.controlMaxClients == 5,            "network.controlMaxClients round-trip");
    check(dst.network.controlBindAddress == "127.0.0.1", "network.controlBindAddress round-trip");
    check(dst.network.controlAllowlist == "127.0.0.1,192.168.1.10",
                                                         "network.controlAllowlist round-trip");
    check(dst.network.controlAuthToken == "s3cr3t-psk",  "network.controlAuthToken round-trip");

    const DisplayPreviewConfig& dp = dst.displayPreview;
    check(dp.enabled == true,                            "displayPreview.enabled round-trip");
    check(dp.device == "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_12:34:56:78:9A:BC-if00",
                                                         "displayPreview.device round-trip");
    check(dp.targetFps == 15,                            "displayPreview.targetFps round-trip");
    check(dp.fit == "stretch",                           "displayPreview.fit round-trip");
    check(dp.jpegQuality == 80,                          "displayPreview.jpegQuality round-trip");
    check(dp.minimumJpegQuality == 30,                   "displayPreview.minimumJpegQuality round-trip");
    check(dp.maxJpegBytes == 32768,                      "displayPreview.maxJpegBytes round-trip");
    check(dp.maxSourceJpegBytes == 2097152,              "displayPreview.maxSourceJpegBytes round-trip");
    check(dp.chunkDataBytes == 2048,                     "displayPreview.chunkDataBytes round-trip");
    check(dp.maxFramesInFlight == 2,                     "displayPreview.maxFramesInFlight round-trip");
    check(dp.assemblyTimeoutMs == 750,                   "displayPreview.assemblyTimeoutMs round-trip");
    check(dp.staleAfterMs == 800,                        "displayPreview.staleAfterMs round-trip");
    check(dp.placeholderAfterMs == 3000,                 "displayPreview.placeholderAfterMs round-trip");
    check(dp.heartbeatMs == 1500,                        "displayPreview.heartbeatMs round-trip");
    check(dp.reconnectMs == 5000,                        "displayPreview.reconnectMs round-trip");
    {
        std::ifstream saved(tmpFile);
        const std::string xml((std::istreambuf_iterator<char>(saved)),
                              std::istreambuf_iterator<char>());
        check(xml.find("NtpStepClock") == std::string::npos &&
              xml.find("NtpStepThresholdSec") == std::string::npos,
              "obsolete SNTP clock-step controls are not serialized");
        check(xml.find("<Encoder") == std::string::npos &&
              xml.find("WarmupFrames") == std::string::npos &&
              xml.find("RecordWidth") == std::string::npos &&
              xml.find("RecordHeight") == std::string::npos,
              "removed encoder/rescale/warm-up settings are not serialized");
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
        check(c1.attributeInfo.size() == 1 && c1.attributeInfo[0].name == "wide_range",
              "cam[1] attributeInfo round-trip");
        if (c1.attributeInfo.size() == 1) {
            const auto& w = c1.attributeInfo[0];
            check(w.minValue == -2147483648.0 && w.maxValue == 2147483647.0 && w.step == 16777217.0,
                  "cam[1] attribute range exact past 2^24 (int32 min/max, step 2^24+1)");
        }
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
    check(cfg.recording.segmentSec == 180, "defaults preserved after failed load");

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
    check(cfg.recording.segmentSec == 180, "defaults preserved after wrong-root load");

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
             "<Recording><SegmentSec>999999</SegmentSec><RetryIntervalSec>0</RetryIntervalSec></Recording>"
             "<Overlay><BackgroundOpacity>5.0</BackgroundOpacity><FontSize>200</FontSize></Overlay>"
             "<Pipeline><CaptureTimeoutMs>-5</CaptureTimeoutMs></Pipeline>"
             "<Log><QueueSize>1</QueueSize></Log></DashcamConfig>";
    }

    std::vector<std::string> warns;
    dashcam::log::LogCallback log = [&](dashcam::log::LogLevel lvl, const std::string& m) {
        if (lvl == dashcam::log::LogLevel::WARN) warns.push_back(m);
    };

    AppConfig cfg;
    check(ConfigReader::load(tmpFile, cfg, log), "load clamp file");
    check(cfg.recording.segmentSec       == 3600, "SegmentSec 999999 -> clamped to max 3600");
    check(cfg.recording.retryIntervalSec == 1,    "RetryIntervalSec 0 -> clamped to min 1");
    check(eq(cfg.overlay.backgroundOpacity, 1.0f), "BackgroundOpacity 5.0 -> clamped to 1.0");
    check(eq(cfg.overlay.fontSize, 72.0f),         "FontSize 200 -> clamped to max 72");
    check(cfg.pipeline.captureTimeoutMs  == 10,   "CaptureTimeoutMs -5 -> clamped to min 10");
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
    check(has("<SegmentSec type=\"int\""), "int field tagged type=int");
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
    check(usbExp && usbExp->gstProperty == "exposure_time_absolute" &&
          usbExp->valueType == AttributeValueType::V4l2Control,
          "exposure/USB -> the exposure_time_absolute V4L2 control");
    const AttributeEntry* csiSat = dict.resolve("saturation", "CSI");
    const AttributeEntry* usbSat = dict.resolve("saturation", "USB");
    check(csiSat && csiSat->valueType == AttributeValueType::Float, "saturation/CSI -> float");
    check(usbSat && usbSat->gstProperty == "saturation" &&
          usbSat->valueType == AttributeValueType::V4l2Control,
          "saturation/USB -> the saturation V4L2 control (v4l2src drops its property while closed)");
    const AttributeEntry* csiAwb = dict.resolve("white balance, automatic", "CSI");
    const AttributeEntry* usbAwb = dict.resolve("white balance, automatic", "USB");
    check(csiAwb && csiAwb->gstProperty == "awblock" && usbAwb &&
          usbAwb->gstProperty == "white_balance_automatic" &&
          usbAwb->valueType == AttributeValueType::V4l2Control,
          "white balance, automatic -> awblock on CSI, the white_balance_automatic control on USB");

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
    check(cfg.recording.segmentSec == 180,        "created config has default segmentSec 180");
    check((std::string)cfg.recording.exposureMode == "auto", "created config has default exposureMode");
    check(eq(cfg.overlay.backgroundOpacity, 0.85f),"created config has default opacity 0.85");
    check(cfg.cameras.empty(),                    "created config has no cameras (pure defaults)");

    // A second call finds the file and loads it (no re-create).
    AppConfig cfg2;
    check(ConfigReader::loadOrCreate(file, cfg2),  "loadOrCreate() returns true when file exists");
    check(cfg2.recording.segmentSec == 180,        "reloaded config matches");

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

// ── test 10: settings of the removed recording path load, ignored, one WARN ───

static bool test_obsolete_settings() {
    std::cout << "\n--- Test 10: obsolete settings ignored with one WARN ---\n";
    const int before = g_fails;

    const std::string tmpFile = "/tmp/libconfig_obsolete_test.xml";
    {
        std::ofstream f(tmpFile);
        if (!f) { std::cerr << "  SKIP  (cannot write temp file)\n"; return true; }
        f << "<?xml version=\"1.0\"?><DashcamConfig>"
             "<Encoder><Bitrate>12000</Bitrate><SpeedPreset>medium</SpeedPreset></Encoder>"
             "<System><FootagePath>/mnt/ssd/footage</FootagePath><WarmupFrames>9</WarmupFrames></System>"
             "<Recording><RecordWidth>1280</RecordWidth><SegmentSec>60</SegmentSec></Recording>"
             "</DashcamConfig>";
    }

    std::vector<std::string> warns;
    dashcam::log::LogCallback log = [&](dashcam::log::LogLevel lvl, const std::string& m) {
        if (lvl == dashcam::log::LogLevel::WARN) warns.push_back(m);
    };

    AppConfig cfg;
    check(ConfigReader::load(tmpFile, cfg, log), "an older file with removed settings still loads");
    check(cfg.system.footagePath == "/mnt/ssd/footage" && cfg.recording.segmentSec == 60,
          "... and its live settings are read");
    check(warns.size() == 1, "exactly one WARN for all of them");
    const std::string w = warns.empty() ? std::string() : warns[0];
    check(w.find("obsolete") != std::string::npos && w.find("<Encoder>") != std::string::npos &&
          w.find("WarmupFrames") != std::string::npos && w.find("RecordWidth") != std::string::npos,
          "the WARN names each obsolete setting present");

    warns.clear();
    AppConfig clean;
    check(ConfigReader::load("config/dashcam.xml", clean, log) && warns.empty(),
          "a current file logs no WARN");

    fs::remove(tmpFile);
    return g_fails == before;
}

// ── DisplayPreview helpers ───────────────────────────────────────────────────

/// Every <DisplayPreview> field name, in declaration (and serialization) order.
static const char* const kPreviewFields[] = {
    "Enabled", "Device", "TargetFps", "Fit", "JpegQuality", "MinimumJpegQuality",
    "MaxJpegBytes", "MaxSourceJpegBytes", "ChunkDataBytes", "MaxFramesInFlight",
    "AssemblyTimeoutMs", "StaleAfterMs", "PlaceholderAfterMs", "HeartbeatMs",
    "ReconnectMs",
};

/// True if every field of @p p still holds its built-in default.
static bool previewIsDefault(const DisplayPreviewConfig& p) {
    auto same = [](const auto& v) { return v == v.defaultValue(); };
    return same(p.enabled) && same(p.device) && same(p.targetFps) && same(p.fit) &&
           same(p.jpegQuality) && same(p.minimumJpegQuality) && same(p.maxJpegBytes) &&
           same(p.maxSourceJpegBytes) && same(p.chunkDataBytes) &&
           same(p.maxFramesInFlight) && same(p.assemblyTimeoutMs) &&
           same(p.staleAfterMs) && same(p.placeholderAfterMs) &&
           same(p.heartbeatMs) && same(p.reconnectMs);
}

/// Write @p body inside a <DashcamConfig> root to @p path; false if it cannot.
static bool writeConfigFile(const std::string& path, const std::string& body) {
    std::ofstream f(path);
    if (!f) return false;
    f << "<?xml version=\"1.0\"?><DashcamConfig>" << body << "</DashcamConfig>";
    return static_cast<bool>(f);
}

// ── test 11: <DisplayPreview> defaults, ranges and the default-config writer ──

static bool test_display_preview_defaults() {
    std::cout << "\n--- Test 11: DisplayPreview defaults + default config file ---\n";
    const int before = g_fails;

    const DisplayPreviewConfig p;
    check(!p.enabled,                         "enabled default false (opt-in)");
    check(p.device == "/dev/dashcam-display", "device default /dev/dashcam-display");
    check(p.fit == "letterbox",               "fit default letterbox");

    // Name, value, default and range of every numeric field, as the plan sets them.
    struct IntSpec { const ConfigVar<int>* v; const char* name; int def, lo, hi; };
    const IntSpec specs[] = {
        {&p.targetFps,          "TargetFps",          10,      1,      15},
        {&p.jpegQuality,        "JpegQuality",        65,      20,     95},
        {&p.minimumJpegQuality, "MinimumJpegQuality", 45,      10,     95},
        {&p.maxJpegBytes,       "MaxJpegBytes",       65536,   4096,   65536},
        {&p.maxSourceJpegBytes, "MaxSourceJpegBytes", 1048576, 262144, 8388608},
        {&p.chunkDataBytes,     "ChunkDataBytes",     1024,    256,    4096},
        {&p.maxFramesInFlight,  "MaxFramesInFlight",  1,       1,      2},
        {&p.assemblyTimeoutMs,  "AssemblyTimeoutMs",  500,     100,    5000},
        {&p.staleAfterMs,       "StaleAfterMs",       500,     100,    9900},
        {&p.placeholderAfterMs, "PlaceholderAfterMs", 2000,    200,    10000},
        {&p.heartbeatMs,        "HeartbeatMs",        1000,    200,    4000},
        {&p.reconnectMs,        "ReconnectMs",        1000,    200,    60000},
    };
    for (const auto& s : specs) {
        const bool ok = s.v->name() == s.name && *s.v == s.def &&
                        s.v->defaultValue() == s.def && s.v->minValue() == s.lo &&
                        s.v->maxValue() == s.hi && !s.v->description().empty();
        check(ok, (std::string(s.name) + " = " + std::to_string(s.def) + " in [" +
                   std::to_string(s.lo) + ", " + std::to_string(s.hi) + "], described").c_str());
    }
    check(!p.enabled.description().empty() && !p.device.description().empty() &&
          !p.fit.description().empty(), "Enabled / Device / Fit are described");
    check(p.heartbeatMs.maxValue() < 5000, "HeartbeatMs cap stays under the 5 s device session timeout");

    // Every in-range value must also be encodable as START_VIDEO: the ranges may not
    // drift past the frozen video-protocol ceilings (Es3VideoProtocol.h).
    check(p.assemblyTimeoutMs.maxValue()  <= int(es3video::kMaxTimeoutMs) &&
          p.staleAfterMs.maxValue()       <= int(es3video::kMaxTimeoutMs) &&
          p.placeholderAfterMs.maxValue() <= int(es3video::kMaxTimeoutMs),
          "timeout caps within the protocol's kMaxTimeoutMs");
    check(p.staleAfterMs.maxValue() < p.placeholderAfterMs.maxValue(),
          "StaleAfterMs cap below PlaceholderAfterMs cap (placeholder > stale always satisfiable)");
    check(p.maxJpegBytes.maxValue()      <= int(es3video::kMaxJpegBytes) &&
          p.chunkDataBytes.maxValue()    <= int(es3video::kMaxChunkDataBytes) &&
          p.maxFramesInFlight.maxValue() <= int(es3video::kMaxCredits) &&
          p.targetFps.maxValue()         <= int(es3video::kMaxTargetFps),
          "JPEG / chunk / credit / fps caps within the protocol ceilings");

    // write_default_config is ConfigReader::save() of a default AppConfig, so the
    // section reaches every build's seeded config/dashcam.xml through the writer.
    const std::string tmpFile = "/tmp/libconfig_preview_default_test.xml";
    check(ConfigReader::save(tmpFile, AppConfig{}), "save() of a default AppConfig");
    std::ifstream in(tmpFile);
    const std::string xml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    const auto open  = xml.find("<DisplayPreview>");
    const auto close = xml.find("</DisplayPreview>");
    check(open != std::string::npos && close != std::string::npos && open < close,
          "default file has a <DisplayPreview> section");
    const std::string section = (open != std::string::npos && close != std::string::npos && open < close)
                                ? xml.substr(open, close - open) : std::string();
    bool allTags = true;
    for (const char* f : kPreviewFields)
        allTags = allTags && section.find("<" + std::string(f) + " type=\"") != std::string::npos;
    check(allTags, "... carrying every field, self-described (type/default/range attributes)");
    check(section.find("default=\"false\">false</Enabled>") != std::string::npos,
          "... with Enabled false");
    check(section.find("<MaxSourceJpegBytes type=\"int\"") != std::string::npos &&
          section.find("max=\"8388608\"") != std::string::npos,
          "... and int ranges written as plain ints");

    AppConfig reloaded;
    std::vector<std::string> problems;
    check(ConfigReader::load(tmpFile, reloaded) && previewIsDefault(reloaded.displayPreview),
          "the default file loads back to the defaults");
    check(validateDisplayPreview(reloaded.displayPreview, problems), "the defaults validate");

    fs::remove(tmpFile);
    return g_fails == before;
}

// ── test 12: an absent <DisplayPreview> section keeps the preview disabled ────

static bool test_display_preview_absent() {
    std::cout << "\n--- Test 12: absent DisplayPreview section keeps defaults ---\n";
    const int before = g_fails;

    std::vector<std::string> warns;
    dashcam::log::LogCallback log = [&](dashcam::log::LogLevel lvl, const std::string& m) {
        if (lvl == dashcam::log::LogLevel::WARN) warns.push_back(m);
    };

    // The shape of every config file already on the car: other sections, no preview.
    const std::string tmpFile = "/tmp/libconfig_preview_absent_test.xml";
    if (!writeConfigFile(tmpFile,
            "<Recording><SegmentSec>120</SegmentSec></Recording>"
            "<Network><StreamEnabled>true</StreamEnabled></Network>")) {
        std::cerr << "  SKIP  (cannot write temp file)\n";
        return true;
    }
    AppConfig cfg;
    check(ConfigReader::load(tmpFile, cfg, log), "an older file without the section loads");
    check(cfg.recording.segmentSec == 120 && cfg.network.streamEnabled,
          "... and its other sections are read");
    check(previewIsDefault(cfg.displayPreview), "every DisplayPreview field keeps its default");
    check(!cfg.displayPreview.enabled, "... so the preview is disabled");
    check(warns.empty(), "a missing section is not worth a WARN");

    // An empty section and a partial one behave the same for the omitted fields.
    check(writeConfigFile(tmpFile, "<DisplayPreview/>"), "write empty-section file");
    AppConfig empty;
    check(ConfigReader::load(tmpFile, empty, log) && previewIsDefault(empty.displayPreview),
          "an empty <DisplayPreview/> keeps every default");

    check(writeConfigFile(tmpFile, "<DisplayPreview><Enabled>true</Enabled></DisplayPreview>"),
          "write enable-only file");
    AppConfig partial;
    std::vector<std::string> problems;
    check(ConfigReader::load(tmpFile, partial, log) && partial.displayPreview.enabled &&
          partial.displayPreview.device == "/dev/dashcam-display" &&
          partial.displayPreview.targetFps == 10 && partial.displayPreview.maxFramesInFlight == 1,
          "Enabled alone turns it on with the other defaults");
    check(validateDisplayPreview(partial.displayPreview, problems), "... and that validates");
    check(warns.empty(), "no WARN for any of them");

    fs::remove(tmpFile);
    return g_fails == before;
}

// ── test 13: out-of-range <DisplayPreview> values clamp, one WARN each ────────

static bool test_display_preview_clamping() {
    std::cout << "\n--- Test 13: DisplayPreview clamping + WARN ---\n";
    const int before = g_fails;

    const std::string tmpFile = "/tmp/libconfig_preview_clamp_test.xml";
    if (!writeConfigFile(tmpFile,
            "<DisplayPreview>"
            "<Enabled>true</Enabled><Device>/dev/dashcam-display</Device><Fit>letterbox</Fit>"
            "<TargetFps>60</TargetFps><JpegQuality>5</JpegQuality>"
            "<MinimumJpegQuality>100</MinimumJpegQuality><MaxJpegBytes>1000000</MaxJpegBytes>"
            "<MaxSourceJpegBytes>1024</MaxSourceJpegBytes><ChunkDataBytes>8192</ChunkDataBytes>"
            "<MaxFramesInFlight>4</MaxFramesInFlight><AssemblyTimeoutMs>0</AssemblyTimeoutMs>"
            "<StaleAfterMs>50000</StaleAfterMs><PlaceholderAfterMs>10</PlaceholderAfterMs>"
            "<HeartbeatMs>5000</HeartbeatMs><ReconnectMs>-1</ReconnectMs>"
            "</DisplayPreview>")) {
        std::cerr << "  SKIP  (cannot write temp file)\n";
        return true;
    }

    std::vector<std::string> warns;
    dashcam::log::LogCallback log = [&](dashcam::log::LogLevel lvl, const std::string& m) {
        if (lvl == dashcam::log::LogLevel::WARN) warns.push_back(m);
    };
    AppConfig cfg;
    check(ConfigReader::load(tmpFile, cfg, log), "load clamp file");
    const DisplayPreviewConfig& p = cfg.displayPreview;
    check(p.targetFps == 15,             "TargetFps 60 -> clamped to max 15");
    check(p.jpegQuality == 20,           "JpegQuality 5 -> clamped to min 20");
    check(p.minimumJpegQuality == 95,    "MinimumJpegQuality 100 -> clamped to max 95");
    check(p.maxJpegBytes == 65536,       "MaxJpegBytes 1000000 -> clamped to max 65536");
    check(p.maxSourceJpegBytes == 262144, "MaxSourceJpegBytes 1024 -> clamped to min 262144");
    check(p.chunkDataBytes == 4096,      "ChunkDataBytes 8192 -> clamped to max 4096");
    check(p.maxFramesInFlight == 2,      "MaxFramesInFlight 4 -> clamped to max 2");
    check(p.assemblyTimeoutMs == 100,    "AssemblyTimeoutMs 0 -> clamped to min 100");
    check(p.staleAfterMs == 9900,        "StaleAfterMs 50000 -> clamped to max 9900");
    check(p.placeholderAfterMs == 200,   "PlaceholderAfterMs 10 -> clamped to min 200");
    check(p.heartbeatMs == 4000,         "HeartbeatMs 5000 -> clamped to max 4000 (under the 5 s timeout)");
    check(p.reconnectMs == 200,          "ReconnectMs -1 -> clamped to min 200");
    check(p.enabled && p.device == "/dev/dashcam-display" && p.fit == "letterbox",
          "in-range fields are read unchanged");

    check(warns.size() == 12, "exactly one WARN per clamped field (12)");
    bool named = true;
    for (const char* f : {"TargetFps", "JpegQuality", "MinimumJpegQuality", "MaxJpegBytes",
                          "MaxSourceJpegBytes", "ChunkDataBytes", "MaxFramesInFlight",
                          "AssemblyTimeoutMs", "StaleAfterMs", "PlaceholderAfterMs",
                          "HeartbeatMs", "ReconnectMs"}) {
        const std::string tag = "'" + std::string(f) + "'";
        bool found = false;
        for (const auto& w : warns) found = found || w.find(tag) != std::string::npos;
        named = named && found;
    }
    check(named, "each WARN names its field");

    // Each value is now in range, yet the pair rules fail: what clamping cannot fix.
    std::vector<std::string> problems;
    check(!validateDisplayPreview(p, problems) && problems.size() == 2,
          "clamped quality and timing pairs still fail validation (2 problems)");

    // Just over the protocol's 10000 ms timeout ceiling: both clamp to their caps,
    // and the result is a valid, START_VIDEO-encodable pair.
    check(writeConfigFile(tmpFile,
              "<DisplayPreview><Enabled>true</Enabled>"
              "<StaleAfterMs>10000</StaleAfterMs><PlaceholderAfterMs>10001</PlaceholderAfterMs>"
              "</DisplayPreview>"),
          "write timeout-ceiling file");
    warns.clear();
    AppConfig edge;
    check(ConfigReader::load(tmpFile, edge, log) && warns.size() == 2,
          "Stale 10000 / Placeholder 10001 load with one WARN each");
    check(edge.displayPreview.staleAfterMs == 9900 && edge.displayPreview.placeholderAfterMs == 10000,
          "... clamped to 9900 / 10000");
    check(validateDisplayPreview(edge.displayPreview, problems) &&
          edge.displayPreview.placeholderAfterMs <= int(es3video::kMaxTimeoutMs),
          "... which validates and stays within the protocol ceiling");

    fs::remove(tmpFile);
    return g_fails == before;
}

// ── test 14: validateDisplayPreview() cross-field rules ──────────────────────

static bool test_display_preview_validation() {
    std::cout << "\n--- Test 14: validateDisplayPreview cross-field rules ---\n";
    const int before = g_fails;

    auto mentions = [](const std::vector<std::string>& v, const char* what) {
        return v.size() == 1 && v[0].find(what) != std::string::npos;
    };
    std::vector<std::string> problems{"stale entry from an earlier call"};

    DisplayPreviewConfig off;
    check(validateDisplayPreview(off, problems) && problems.empty(),
          "defaults (disabled) are valid; the problems list is cleared");
    DisplayPreviewConfig on;
    on.enabled = true;
    check(validateDisplayPreview(on, problems) && problems.empty(), "defaults enabled are valid");

    DisplayPreviewConfig fit = on;
    fit.fit = "stretch";
    check(!validateDisplayPreview(fit, problems) && mentions(problems, "Fit"),
          "Fit 'stretch' is rejected, naming Fit");
    fit.fit = "Letterbox";
    check(!validateDisplayPreview(fit, problems), "Fit is matched exactly ('Letterbox' rejected)");
    fit.fit = "";
    check(!validateDisplayPreview(fit, problems), "an empty Fit is rejected");

    DisplayPreviewConfig q = on;
    q.jpegQuality = 65;
    q.minimumJpegQuality = 65;
    check(validateDisplayPreview(q, problems), "MinimumJpegQuality == JpegQuality is valid (no retry headroom)");
    q.minimumJpegQuality = 66;
    check(!validateDisplayPreview(q, problems) && mentions(problems, "MinimumJpegQuality"),
          "MinimumJpegQuality > JpegQuality is rejected, naming it");

    DisplayPreviewConfig t = on;
    t.staleAfterMs = 500;
    t.placeholderAfterMs = 500;
    check(!validateDisplayPreview(t, problems) && mentions(problems, "PlaceholderAfterMs"),
          "PlaceholderAfterMs == StaleAfterMs is rejected, naming it");
    t.placeholderAfterMs = 501;
    check(validateDisplayPreview(t, problems), "PlaceholderAfterMs > StaleAfterMs is valid");

    DisplayPreviewConfig dev = on;
    dev.device = "";
    check(!validateDisplayPreview(dev, problems) && mentions(problems, "Device"),
          "Enabled with an empty Device is rejected (no port probing)");
    dev.enabled = false;
    check(validateDisplayPreview(dev, problems), "an empty Device is fine while disabled");

    DisplayPreviewConfig all = on;
    all.fit = "crop";
    all.jpegQuality = 40;
    all.minimumJpegQuality = 60;
    all.staleAfterMs = 3000;
    all.placeholderAfterMs = 2000;
    all.device = "";
    check(!validateDisplayPreview(all, problems) && problems.size() == 4,
          "every violated rule is reported at once (4)");
    for (const auto& line : problems) std::cout << "        " << line << "\n";

    return g_fails == before;
}

// ── test 15: <Navigation> defaults, round trip, absence, clamping, rules ─────

static bool test_navigation() {
    std::cout << "\n--- Test 15: Navigation section ---\n";
    const int before = g_fails;
    std::vector<std::string> warns;
    dashcam::log::LogCallback log = [&](dashcam::log::LogLevel lvl, const std::string& m) {
        if (lvl == dashcam::log::LogLevel::WARN) warns.push_back(m);
    };
    const NavigationConfig d;
    std::vector<std::string> problems;
    check(!d.enabled && d.zoom == 16 && d.maxFps == 4 && d.warmFps == 1 && d.jpegQuality == 80 &&
              d.cacheBudgetMB == 256 && d.gpsStaleMs == 1500 && d.staleAfterMs == 2000 &&
              d.placeholderAfterMs == 5000,
          "defaults: off, zoom 16, 4 fps, warm 1 fps, q80, 256 MiB, GPS stale 1.5 s, 2 s / 5 s");
    check(std::string(d.providerUrl) == "https://tile.openstreetmap.org/{z}/{x}/{y}.png" &&
              std::string(d.attribution) == "\xC2\xA9 OpenStreetMap contributors",
          "OSM provider and its attribution (UTF-8 copyright sign)");
    check(validateNavigation(d, problems), "the defaults validate");
    check(d.placeholderAfterMs.maxValue() <= es3video::kMaxTimeoutMs && d.staleAfterMs.maxValue() < d.placeholderAfterMs.maxValue() &&
              d.maxFps.maxValue() <= es3video::kMaxTargetFps,
          "map timeouts and fps fit the display protocol (START_MAP shares the video limits)");

    const std::string tmpFile = "/tmp/libconfig_navigation_test.xml";
    AppConfig out;
    out.navigation.enabled     = true;
    out.navigation.providerUrl = std::string("http://127.0.0.1:8080/t/{z}/{x}/{y}.png");
    out.navigation.zoom        = 17;
    out.navigation.warmFps     = 0;
    out.navigation.cachePath   = std::string("/tmp/navcache.sqlite");
    out.navigation.panning     = false;
    out.navigation.poseHz      = 25;
    out.navigation.panBackgroundWidth  = 448;
    out.navigation.panBackgroundHeight = 352;
    check(ConfigReader::save(tmpFile, out, log), "save");
    AppConfig in;
    check(ConfigReader::load(tmpFile, in, log) && in.navigation.enabled && in.navigation.zoom == 17 &&
              in.navigation.warmFps == 0 && std::string(in.navigation.cachePath) == "/tmp/navcache.sqlite" &&
              std::string(in.navigation.attribution) == std::string(d.attribution),
          "round trip (incl. the UTF-8 attribution)");
    check(!in.navigation.panning && in.navigation.poseHz == 25 && in.navigation.panBackgroundWidth == 448 &&
              in.navigation.panBackgroundHeight == 352 && in.navigation.panJpegQuality == 75,
          "panning settings round trip");
    check(d.panning && d.poseHz == 20 && d.panBackgroundWidth == 512 && d.panBackgroundHeight == 384 &&
              d.panBackgroundWidth.maxValue() <= 1024 && d.poseHz.maxValue() <= 60 && d.panBackgroundWidth.minValue() >= 320,
          "panning defaults (on, 20 Hz, 512x384) fit map extension v2's ceilings");
    check(validateNavigation(in.navigation, problems), "a localhost http provider validates (test servers)");

    check(writeConfigFile(tmpFile, "<Recording><SegmentSec>60</SegmentSec></Recording>"), "write file w/o section");
    AppConfig old;
    check(ConfigReader::load(tmpFile, old, log) && !old.navigation.enabled && old.navigation.zoom == 16,
          "a file without <Navigation> keeps it off");

    warns.clear();
    check(writeConfigFile(tmpFile, "<Navigation><Zoom>21</Zoom><MaxFps>0</MaxFps>"
                                   "<PlaceholderAfterMs>60000</PlaceholderAfterMs></Navigation>"),
          "write out-of-range file");
    AppConfig cl;
    check(ConfigReader::load(tmpFile, cl, log) && cl.navigation.zoom == 18 && cl.navigation.maxFps == 1 &&
              cl.navigation.placeholderAfterMs == 10000 && warns.size() == 3,
          "zoom 21 -> 18, fps 0 -> 1, placeholder 60000 -> 10000, one WARN each");

    NavigationConfig bad;
    bad.providerUrl        = std::string("http://tile.example.org/{z}/{y}.png");
    bad.userAgent          = std::string("");
    bad.attribution        = std::string("");
    bad.staleAfterMs       = 3000;
    bad.placeholderAfterMs = 3000;
    check(!validateNavigation(bad, problems) && problems.size() == 5,
          "plain http, missing {x}, empty UserAgent and Attribution, placeholder <= stale: 5 problems");
    auto only = [&](const NavigationConfig& c, const char* needle) {
        return !validateNavigation(c, problems) && problems.size() == 1 && problems[0].find(needle) != std::string::npos;
    };
    NavigationConfig u = d;
    u.providerUrl = std::string("https://{s}.tile.example.org/{z}/{x}/{y}.png");
    check(only(u, "{s}"), "an unsupported placeholder ({s}) is rejected by name");
    u.providerUrl = std::string("http://localhost.example.com/{z}/{x}/{y}.png");
    check(only(u, "https"), "http to localhost.example.com is not 'localhost'");
    u.providerUrl = std::string("http://user@localhost/{z}/{x}/{y}.png");
    check(only(u, "https"), "http with user info is rejected");
    u.providerUrl = std::string("http://localhost:8080/{z}/{x}/{y}.png");
    check(validateNavigation(u, problems), "http://localhost:8080 is a local test server");
    u.providerUrl = std::string("http://[::1]:9/{z}/{x}/{y}.png");
    check(validateNavigation(u, problems), "http://[::1] too");
    u.providerUrl = std::string("http://tiles.example.org/{z}/{x}/{y}.png?key=SECRET123");
    check(!validateNavigation(u, problems) && problems[0].find("SECRET123") == std::string::npos,
          "the message never repeats the URL's query (keys)");
    NavigationConfig at = d;
    at.attribution = std::string("\xC2\xA9 OpenStreetMap contributors, \xC2\xA9 A Very Long Tile Provider Name Ltd");
    check(only(at, "Attribution"), "an attribution too long for the strip is rejected");
    check(d.staleAfterMs.minValue() >= 1500, "StaleAfterMs below the map's refresh (+ margin) is clamped up to 1500");
    check(d.panBackgroundWidth.minValue() >= 384 && d.panBackgroundHeight.minValue() >= 256,
          "panning backgrounds at least the map area + 64 (384x256)");
    {
        warns.clear();
        check(writeConfigFile(tmpFile, "<Navigation><StaleAfterMs>1000</StaleAfterMs><PanBackgroundWidth>320"
                                       "</PanBackgroundWidth><PanBackgroundHeight>192</PanBackgroundHeight></Navigation>"),
              "write a file with old-range values");
        AppConfig oldRange;
        check(ConfigReader::load(tmpFile, oldRange, log) && oldRange.navigation.staleAfterMs == 1500 &&
                  oldRange.navigation.panBackgroundWidth == 384 && oldRange.navigation.panBackgroundHeight == 256 &&
                  warns.size() == 3 && validateNavigation(oldRange.navigation, problems),
              "old-range values are clamped with a WARN each and the map stays valid");
    }
    DisplayPreviewConfig dl;
    check(validateDisplayLink(dl, problems) == !std::string(dl.device).empty(), "the link defaults");
    dl.device             = std::string("");
    dl.staleAfterMs       = 3000;
    dl.placeholderAfterMs = 2000;
    check(!validateDisplayLink(dl, problems) && problems.size() == 2,
          "map-only config: an empty Device and placeholder <= stale in <DisplayPreview> are reported");
    fs::remove(tmpFile);
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
    bool t10 = test_obsolete_settings();
    bool t11 = test_display_preview_defaults();
    bool t12 = test_display_preview_absent();
    bool t13 = test_display_preview_clamping();
    bool t14 = test_display_preview_validation();
    bool t15 = test_navigation();

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
    std::cout << "Test 10 (obsolete settings): " << (t10 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 11 (preview defaults): " << (t11 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 12 (preview absent):   " << (t12 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 13 (preview clamping): " << (t13 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 14 (preview validate): " << (t14 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 15 (navigation):       " << (t15 ? "PASS" : "FAIL") << "\n";

    return (t1 && t2 && t3 && t4 && t5 && t6 && t7 && t8 && t9 && t10 &&
            t11 && t12 && t13 && t14 && t15) ? 0 : 1;
}
