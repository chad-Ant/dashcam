// lane_status — the lane test's status display (plan P5b, section 12.4 alert
// channel): shows RUN_DIR/status.json (written at 1 Hz by
// tools/lane_eval/lane_supervisor.py) on the CYD / ES3C28P over libes3display.
//
//   lane_status --device DEV --status-file RUN/status.json [--page 0] [--alert-volume 80] [--minutes N]
//
// The supervisor composes the text (overall GREEN / AMBER / RED, a title and
// four 23-character lines: disp_title, disp_l1..disp_l4); this program only
// carries it: a PAGE update when the text changes (and every 5 s), and an
// ALERT (the board's /alert.wav) on entering RED and every 60 s while RED.  A
// status file older than 5 s shows as RED "NO STATUS".  Controls only (no
// video), so it works with both boards' firmware.
//
// A separate binary from lane_drive on purpose: libes3display pulls in libjpeg
// (libvideopreview), and a libjpeg in lane_drive's global symbol scope breaks
// nvjpegenc ("JPEG parameter struct mismatch").  SIGINT / SIGTERM: exit 0.

#include "libes3display.h"
#include "liblog.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>

namespace {

std::atomic<int> g_signal{0};
void onSignal(int sig) { g_signal = sig; }

int64_t monoNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

std::string readSmall(const std::string& path) {
    std::ifstream f(path);
    if (!f) return std::string();
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool jsonNumber(const std::string& j, const std::string& key, double& v) {
    const auto p = j.find("\"" + key + "\"");
    if (p == std::string::npos) return false;
    const auto c = j.find(':', p);
    if (c == std::string::npos) return false;
    const char* s = j.c_str() + c + 1;
    while (*s == ' ') ++s;
    char* end = nullptr;
    v = std::strtod(s, &end);
    return end != s;
}

bool jsonString(const std::string& j, const std::string& key, std::string& v) {
    const auto p = j.find("\"" + key + "\"");
    if (p == std::string::npos) return false;
    const auto q = j.find('"', j.find(':', p) + 1);
    if (q == std::string::npos) return false;
    v.clear();
    for (size_t i = q + 1; i < j.size(); ++i) {
        if (j[i] == '\\' && i + 1 < j.size()) { v += j[++i]; continue; }
        if (j[i] == '"') return true;
        v += j[i];
    }
    return false;
}

/// At most 23 printable ASCII characters (the board's PAGE line).
std::string fit(std::string s) {
    if (s.size() > 23) s.resize(23);
    for (char& c : s) if (c < 32 || c > 126) c = '?';
    return s;
}

[[noreturn]] void usage(const char* msg) {
    std::cerr << "lane_status: " << msg << "\n"
              << "usage: lane_status --device DEV --status-file FILE [--page 0] [--alert-volume 80] [--minutes N]\n";
    std::exit(1);
}

} // namespace

int main(int argc, char** argv) {
    std::string dev, file;
    int page = 0, volume = 80;
    double minutes = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage(("missing value for " + a).c_str());
            return argv[++i];
        };
        if (a == "--device") dev = next();
        else if (a == "--status-file") file = next();
        else if (a == "--page") page = std::stoi(next());
        else if (a == "--alert-volume") volume = std::stoi(next());
        else if (a == "--minutes") minutes = std::stod(next());
        else usage(("unknown option " + a).c_str());
    }
    if (dev.empty() || file.empty()) usage("--device and --status-file are required");
    if (page < 0 || page >= dashcam::es3display::kPageCount) usage("--page 0..4");
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    namespace es3 = dashcam::es3display;
    es3::DisplayLinkConfig dc;
    dc.device = dev;
    dc.video = false;
    // Slow reconnects: opening the CYD's CH340 port can toggle the ESP32's
    // auto-reset lines, and a board without the ES3 protocol (the CYD's
    // dashboard firmware of 2026-10-13) never answers HELLO.
    dc.reconnectMs = 10000;
    es3::Es3DisplayLink link;
    auto log = [](dashcam::log::LogLevel lvl, const std::string& msg) {
        if (lvl != dashcam::log::LogLevel::DEBUG) std::fprintf(stderr, "[lane_status] %s\n", msg.c_str());
    };
    if (!link.start(dc, log)) { std::fprintf(stderr, "[lane_status] bad display config\n"); return 1; }

    constexpr int64_t kSec = 1'000'000'000LL;
    const int64_t end = minutes > 0 ? monoNs() + static_cast<int64_t>(minutes * 60e9) : INT64_MAX;
    std::string lastText, lastLevel = "GREEN";
    int64_t lastPage = 0, lastAlert = 0, lastReport = 0;
    std::atomic<uint32_t> pagesOk{0}, pagesBad{0}, alertsOk{0}, alertsBad{0};
    bool wasConnected = false;
    while (!g_signal && monoNs() < end) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const int64_t now = monoNs();
        const auto st = link.stats();
        if (st.connected != wasConnected) {
            std::fprintf(stderr, "[lane_status] %s %s (%s)\n", st.connected ? "connected" : "disconnected",
                         dev.c_str(), st.identity.c_str());
            wasConnected = st.connected;
        }
        const std::string j = readSmall(file);
        std::string level = "AMBER", title = "LANE TEST", l[4];
        jsonString(j, "overall", level);
        jsonString(j, "disp_title", title);
        for (int k = 0; k < 4; ++k) jsonString(j, "disp_l" + std::to_string(k + 1), l[k]);
        double mono = 0;
        if (j.empty() || !jsonNumber(j, "mono_ns", mono) || now - static_cast<int64_t>(mono) > 5 * kSec) {
            level = "RED";
            title = "NO STATUS";
            l[0] = "supervisor silent";
            l[1] = l[2] = l[3] = "";
        }
        const std::string head = fit(level + " " + title);
        const std::string text = head + "|" + fit(l[0]) + "|" + fit(l[1]) + "|" + fit(l[2]) + "|" + fit(l[3]);
        if (st.connected && (text != lastText || now - lastPage > 5 * kSec)) {
            auto req = es3::ControlRequest::page(static_cast<uint8_t>(page), head,
                                                 { fit(l[0]), fit(l[1]), fit(l[2]), fit(l[3]) });
            req.onDone = [&pagesOk, &pagesBad](const es3::ControlResult& r) {
                if (r.status == es3::ControlStatus::Ok) ++pagesOk; else ++pagesBad;
            };
            if (link.enqueueControl(req)) { lastText = text; lastPage = now; }
        }
        // Beep on entering RED and every 60 s while it stays RED.
        if (st.connected && level == "RED" && (lastLevel != "RED" || now - lastAlert > 60 * kSec)) {
            auto req = es3::ControlRequest::alert(static_cast<uint8_t>(std::clamp(volume, 0, 100)));
            req.onDone = [&alertsOk, &alertsBad](const es3::ControlResult& r) {
                if (r.status == es3::ControlStatus::Ok) ++alertsOk; else ++alertsBad;
            };
            if (link.enqueueControl(req)) lastAlert = now;
        }
        if (st.connected) lastLevel = level;
        if (now - lastReport > 30 * kSec) {
            lastReport = now;
            std::fprintf(stderr, "[lane_status] %s connected %d pages ok %u failed %u alerts ok %u failed %u\n",
                         level.c_str(), st.connected, pagesOk.load(), pagesBad.load(), alertsOk.load(), alertsBad.load());
        }
    }
    link.stop();
    std::fprintf(stderr, "[lane_status] exit: pages ok %u failed %u alerts ok %u failed %u\n", pagesOk.load(),
                 pagesBad.load(), alertsOk.load(), alertsBad.load());
    return 0;
}
