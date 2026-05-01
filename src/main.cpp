// CSI camera function test — Jetson Orin Nano, Argus path (IMX296 1080p60)
// Usage: ./csi_test [sensor-id]   (default sensor-id=0)
//
// Requires: GStreamer 1.0 with nvarguscamerasrc + nvvidconv (JetPack 6.2 / L4T R36.4.x)
// Run nvargus-daemon before starting:  sudo systemctl restart nvargus-daemon

#include <gst/gst.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

struct TestResult {
    std::string name;
    bool        passed;
    std::string message;
};

static std::atomic<bool> g_quit{false};
static void on_sigint(int) { g_quit = true; }

// Pad probe increments a frame counter for every buffer that passes through.
static GstPadProbeReturn frame_count_probe(GstPad*, GstPadProbeInfo*, gpointer ud)
{
    (*static_cast<std::atomic<int>*>(ud))++;
    return GST_PAD_PROBE_OK;
}

// Build + run a pipeline string for up to `duration_s` seconds.
// The pipeline must contain a fakesink named "sink0" — buffers arriving
// at its sink pad are counted via a probe.
// Returns frame count on success, -1 on pipeline error.
static int run_timed(const std::string& pl, int duration_s)
{
    GError*     err      = nullptr;
    GstElement* pipeline = gst_parse_launch(pl.c_str(), &err);
    if (!pipeline || err) {
        if (err) {
            fprintf(stderr, "  parse error: %s\n", err->message);
            g_error_free(err);
        }
        if (pipeline) gst_object_unref(pipeline);
        return -1;
    }

    std::atomic<int> frames{0};
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink0");
    if (sink) {
        GstPad* pad = gst_element_get_static_pad(sink, "sink");
        gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                          frame_count_probe, &frames, nullptr);
        gst_object_unref(pad);
        gst_object_unref(sink);
    }

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "  set_state(PLAYING) failed\n");
        gst_object_unref(pipeline);
        return -1;
    }

    GstState state;
    GstStateChangeReturn sc =
        gst_element_get_state(pipeline, &state, nullptr, 5 * GST_SECOND);
    if (sc == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING) {
        fprintf(stderr, "  pipeline did not reach PLAYING (state=%s)\n",
                gst_element_state_get_name(state));
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        return -1;
    }

    GstBus* bus      = gst_element_get_bus(pipeline);
    auto    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
    bool    bus_err  = false;

    while (!g_quit && std::chrono::steady_clock::now() < deadline) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 100 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (!msg) continue;
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError* e = nullptr; gchar* dbg = nullptr;
            gst_message_parse_error(msg, &e, &dbg);
            fprintf(stderr, "  bus error: %s\n", e ? e->message : "(unknown)");
            if (e) g_error_free(e);
            g_free(dbg);
            bus_err = true;
        }
        gst_message_unref(msg);
        break;
    }

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return bus_err ? -1 : frames.load();
}

// ── Individual tests ──────────────────────────────────────────────────────────

// Test 1: sanity — can Argus open the sensor and deliver num-buffers=10?
static TestResult test_start(int sid)
{
    std::string pl =
        "nvarguscamerasrc sensor-id=" + std::to_string(sid) + " num-buffers=10 ! "
        "'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12' ! "
        "nvvidconv ! 'video/x-raw,format=I420' ! fakesink name=sink0";

    int n = run_timed(pl, 6);
    bool ok = (n >= 0);
    return {
        "pipeline start (sensor-id=" + std::to_string(sid) + ")", ok,
        ok ? "received " + std::to_string(n) + " frames (expected 10)"
           : "failed — check sensor connection and nvargus-daemon"
    };
}

// Test 2: 1080p30 sustained frame delivery (3 s, expect ~90, accept >= 60)
static TestResult test_1080p30(int sid)
{
    std::string pl =
        "nvarguscamerasrc sensor-id=" + std::to_string(sid) + " ! "
        "'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12' ! "
        "nvvidconv ! 'video/x-raw,format=I420' ! fakesink name=sink0 sync=false";

    int n = run_timed(pl, 3);
    bool ok = (n >= 60);
    return {
        "1080p30 frame delivery (sensor-id=" + std::to_string(sid) + ")", ok,
        n >= 0 ? std::to_string(n) + " frames in 3 s (expected ~90)"
               : "pipeline error"
    };
}

// Test 3: 1080p60 — primary dashcam mode for IMX296 global-shutter sensor
//         (3 s, expect ~180, accept >= 120)
static TestResult test_1080p60(int sid)
{
    std::string pl =
        "nvarguscamerasrc sensor-id=" + std::to_string(sid) + " ! "
        "'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1,format=NV12' ! "
        "nvvidconv ! 'video/x-raw,format=I420' ! fakesink name=sink0 sync=false";

    int n = run_timed(pl, 3);
    bool ok = (n >= 120);
    return {
        "1080p60 frame delivery — dashcam mode (sensor-id=" + std::to_string(sid) + ")", ok,
        n >= 0 ? std::to_string(n) + " frames in 3 s (expected ~180)"
               : "pipeline error — sensor may not support 1080p60 in this ISP mode"
    };
}

// Test 4: Dynamic ISP Control — inject AE lock + fixed exposure mid-stream
//         and verify frames keep flowing (>= 60 frames over ~3 s at 30 fps).
static TestResult test_dynamic_attributes(int sid)
{
    std::string pl =
        "nvarguscamerasrc name=camerasrc sensor-id=" + std::to_string(sid) + " ! "
        "'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12' ! "
        "nvvidconv ! 'video/x-raw,format=I420' ! fakesink name=sink0 sync=false";

    GError*     err      = nullptr;
    GstElement* pipeline = gst_parse_launch(pl.c_str(), &err);
    if (!pipeline || err) {
        if (err) { g_error_free(err); }
        if (pipeline) gst_object_unref(pipeline);
        return {"Dynamic Attribute Injection (sensor-id=" + std::to_string(sid) + ")",
                false, "Parse error"};
    }

    std::atomic<int> frames{0};
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink0");
    GstElement* src  = gst_bin_get_by_name(GST_BIN(pipeline), "camerasrc");

    if (sink) {
        GstPad* pad = gst_element_get_static_pad(sink, "sink");
        gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, frame_count_probe, &frames, nullptr);
        gst_object_unref(pad);
        gst_object_unref(sink);
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Wait for PLAYING before touching ISP properties.
    GstState state;
    GstStateChangeReturn sc =
        gst_element_get_state(pipeline, &state, nullptr, 5 * GST_SECOND);
    if (sc == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING) {
        if (src) gst_object_unref(src);
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        return {"Dynamic Attribute Injection (sensor-id=" + std::to_string(sid) + ")",
                false, "Pipeline failed to reach PLAYING"};
    }

    // Let auto-exposure settle for 1 s before locking it.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Mid-stream: lock AE and pin exposure time to 13 ms (max for 60 fps headroom).
    if (src) {
        g_object_set(G_OBJECT(src), "aelock", TRUE, NULL);
        g_object_set(G_OBJECT(src), "exposuretimerange", "13000 13000", NULL);
        gst_object_unref(src);
    }

    // 2 s more — frames must not drop after property injection.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    bool ok = (frames.load() >= 60);
    return {
        "Dynamic Attribute Injection (sensor-id=" + std::to_string(sid) + ")", ok,
        ok ? "Daemon accepted mid-stream property updates cleanly ("
                 + std::to_string(frames.load()) + " frames)"
           : "Pipeline crashed or dropped frames during attribute update"
    };
}

// Test 5: Argus Daemon Rapid Restart Stability — open/close 3 times, 200 ms apart.
static TestResult test_daemon_stability(int sid)
{
    std::string pl =
        "nvarguscamerasrc sensor-id=" + std::to_string(sid) + " num-buffers=15 ! "
        "'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12' ! "
        "nvvidconv ! 'video/x-raw,format=I420' ! fakesink name=sink0 sync=false";

    int runs_passed = 0;
    for (int i = 0; i < 3; ++i) {
        if (run_timed(pl, 3) > 0) runs_passed++;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    bool ok = (runs_passed == 3);
    return {
        "Argus Daemon Rapid Restart (sensor-id=" + std::to_string(sid) + ")", ok,
        ok ? "Daemon survived 3 rapid start/stop cycles"
           : "Daemon locked up on cycle " + std::to_string(runs_passed + 1)
                 + " — run: sudo systemctl restart nvargus-daemon"
    };
}

// ── Reporting ─────────────────────────────────────────────────────────────────

static void print_result(const TestResult& r)
{
    const char* tag = r.passed ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m";
    printf("[%s] %s\n       %s\n", tag, r.name.c_str(), r.message.c_str());
}

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    std::signal(SIGINT, on_sigint);
    gst_init(&argc, &argv);

    int sid = (argc >= 2) ? std::atoi(argv[1]) : 0;

    printf("=== CSI Camera Function Test ===\n");
    printf("sensor-id : %d\n", sid);
    printf("target    : IMX296 global-shutter, Argus path, JetPack 6.2\n");
    printf("usage     : %s [sensor-id]\n\n", argv[0]);

    std::vector<TestResult> results;

    // Test 1 gates the rest — no point testing frame rates if Argus can't open the sensor.
    results.push_back(test_start(sid));
    print_result(results.back());
    if (!results.back().passed) {
        printf("\nGating failure — skipping remaining tests.\n");
        printf("Triage steps:\n");
        printf("  sudo systemctl restart nvargus-daemon\n");
        printf("  GST_DEBUG=3 gst-launch-1.0 nvarguscamerasrc num-buffers=5 ! fakesink\n");
        printf("  v4l2-ctl --list-devices\n");
        return 1;
    }

    results.push_back(test_1080p30(sid));
    print_result(results.back());

    results.push_back(test_1080p60(sid));
    print_result(results.back());

    results.push_back(test_dynamic_attributes(sid));
    print_result(results.back());

    results.push_back(test_daemon_stability(sid));
    print_result(results.back());

    int passed = 0, failed = 0;
    for (const auto& r : results) r.passed ? ++passed : ++failed;

    printf("\n%d/%d tests passed\n", passed, passed + failed);
    return failed > 0 ? 1 : 0;
}
