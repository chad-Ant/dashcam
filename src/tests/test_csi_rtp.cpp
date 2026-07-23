// CSI → Argus → x264 → RTP hardware smoke test.
//
// Proves the inference-camera RTP path actually carries H.264 end-to-end on real
// hardware: it opens the IMX296 CSI camera through Argus, attaches libnetwork's
// RtpSession branch (nvvidconv → x264enc → rtph264pay → udpsink) to a loopback
// destination, and runs a SECOND, in-process GStreamer pipeline as the receiver
// (udpsrc → rtpjitterbuffer → rtph264depay → h264parse → appsink) that counts the
// decoded access units.  Frames received ⇒ the whole nvmm RTP path works.
//
// No display sink, no external process.  Must run where a CSI camera + Argus are
// available (the l4t-ml-gpio container with /tmp/argus_socket mounted).  If no CSI
// camera is discovered the test SKIPS (returns 0) rather than failing, so it is
// safe to invoke in camera-less environments.
//
// Returns 0 on success (or skip), 1 if a discovered CSI camera produced no RTP.

#include "libcamera_csi.h"
#include "libnetwork.h"
#include "liblog.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <vector>

namespace cam = dashcam::camera;
namespace net = dashcam::network;
using LvL = dashcam::log::LogLevel;

static std::atomic<bool> g_stop{false};
static void onSignal(int) { g_stop.store(true); }

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    gst_init(&argc, &argv);

    auto log = dashcam::log::getCallback();
    log(LvL::INFO, "=== CSI RTP hardware smoke test ===");

    // Low output resolution / rate keeps the software x264 encoder light — this
    // is a "does RTP flow" check, not a throughput benchmark.
    const uint16_t    kPort   = 5600;
    const uint32_t    kOutW   = 640, kOutH = 360;
    const float       kOutFps = 15.0f;
    const int         kRunSec = 8;
    const int         kMinFrames = 5;   // ~8 s at 15 fps minus encoder/keyframe warmup

    // ── discover a CSI camera ─────────────────────────────────────────────────
    std::vector<cam::cameraInfo> found;
    if (cam::getCameraList(found, log) != cam::ERROR_CODE::NONE) {
        log(LvL::WARN, "camera discovery failed — skipping");
        dashcam::log::shutdown();
        return 0;
    }
    const cam::cameraInfo* csi = nullptr;
    for (const auto& info : found)
        if (info.type == cam::CAMERA_TYPE::CSI) { csi = &info; break; }
    if (!csi) {
        log(LvL::WARN, "no CSI camera discovered — skipping (needs IMX296 + Argus)");
        dashcam::log::shutdown();
        return 0;
    }
    log(LvL::INFO, "using CSI camera: " + csi->address);

    // ── receiver first, so udpsrc is bound before the sender starts ───────────
    const std::string rxDesc =
        "udpsrc name=src port=" + std::to_string(kPort) +
        " caps=\"application/x-rtp,media=(string)video,clock-rate=(int)90000,"
        "encoding-name=(string)H264,payload=(int)96\""
        " ! rtpjitterbuffer latency=100 ! rtph264depay ! h264parse"
        " ! appsink name=rxsink sync=false max-buffers=8 drop=true";
    GError* err = nullptr;
    GstElement* rx = gst_parse_launch(rxDesc.c_str(), &err);
    if (!rx || err) {
        log(LvL::ERROR, std::string("receiver pipeline failed: ") + (err ? err->message : "?"));
        if (err) g_error_free(err);
        if (rx) gst_object_unref(rx);
        dashcam::log::shutdown();
        return 1;
    }
    GstElement* rxSink = gst_bin_get_by_name(GST_BIN(rx), "rxsink");
    gst_element_set_state(rx, GST_STATE_PLAYING);

    // ── sender: CSI camera + RTP branch ───────────────────────────────────────
    net::RtpStreamConfig rc;
    rc.enabled = true; rc.host = "127.0.0.1"; rc.port = kPort;
    rc.bitrateKbps = 2000; rc.keyIntSec = 1;   // 1 s key-int → quick receiver lock
    net::RtpSession rtp(rc);

    cam::Camera_CSI camdev(*csi);
    camdev.setLogCallback(log);
    camdev.setOutputResolution(kOutW, kOutH, kOutFps);

    const std::string desc = rtp.branchDescription(/*nvmm=*/true, kOutFps, "csi-rtpsink");
    log(LvL::INFO, "RTP branch: " + desc);
    GstElement* rtpBin = gst_parse_bin_from_description(desc.c_str(), TRUE, &err);
    if (!rtpBin || err) {
        log(LvL::ERROR, std::string("RTP branch parse failed: ") + (err ? err->message : "?"));
        if (err) g_error_free(err);
        gst_element_set_state(rx, GST_STATE_NULL);
        if (rxSink) gst_object_unref(rxSink);
        gst_object_unref(rx);
        dashcam::log::shutdown();
        return 1;
    }
    camdev.addBranch("rtp", rtpBin, /*leaky=*/true);

    camdev.open();
    camdev.setCameraVideoFormat(0);
    camdev.start();
    camdev.setCaptureEnabled(false);   // branch-only consumer (no main appsink pull)

    cam::cameraStatus st;
    camdev.getCameraStatus(st);
    int rc_exit = 0;
    if (st.status != cam::CAMERA_STATUS::RUNNING) {
        log(LvL::ERROR, "CSI pipeline did not reach RUNNING (status=" +
                        std::to_string((int)st.status) + " err=" +
                        std::to_string((int)st.currentError) + ") — RTP path FAILED");
        rc_exit = 1;
    } else {
        // ── count received access units for kRunSec ───────────────────────────
        int frames = 0;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(kRunSec);
        while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
            GstSample* smp = gst_app_sink_try_pull_sample(
                GST_APP_SINK(rxSink), 200 * GST_MSECOND);
            if (smp) { ++frames; gst_sample_unref(smp); }
        }
        const bool ok = frames >= kMinFrames;
        log(ok ? LvL::INFO : LvL::ERROR,
            "received " + std::to_string(frames) + " H.264 access unit(s) in " +
            std::to_string(kRunSec) + " s  " + (ok ? "OK" : "FAIL"));
        rc_exit = ok ? 0 : 1;
    }

    // ── teardown ──────────────────────────────────────────────────────────────
    camdev.stop();
    camdev.close();
    gst_element_set_state(rx, GST_STATE_NULL);
    if (rxSink) gst_object_unref(rxSink);
    gst_object_unref(rx);

    log(rc_exit == 0 ? LvL::INFO : LvL::ERROR,
        rc_exit == 0 ? "=== CSI RTP smoke test PASSED ===" : "=== CSI RTP smoke test FAILED ===");
    dashcam::log::shutdown();
    return rc_exit;
}
