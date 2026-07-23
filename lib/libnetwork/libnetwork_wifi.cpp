/**
 * @file libnetwork_wifi.cpp
 * @brief WiFi bring-up via NetworkManager (nmcli), with an offline fallback.
 *
 * connectWifi() asks the OS to connect to a WiFi SSID at startup; if that cannot
 * be fulfilled the caller runs in offline mode.  nmcli is invoked with fork/exec
 * and an argv array — NEVER a shell string — so an SSID containing spaces,
 * quotes, or shell metacharacters cannot inject a command (this deliberately
 * differs from libcan's ::system() approach, whose interface names are trusted).
 *
 * The dashcam runs inside the l4t-ml container; nmcli here talks to the HOST's
 * NetworkManager over the mounted D-Bus socket (see docker_dev/Dockerfile and
 * the launchcode_*.sh mounts).  When nmcli or NetworkManager is unavailable,
 * every call degrades to "offline" rather than failing.
 */

#include "libnetwork.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

namespace dashcam::network {

using LvL = dashcam::log::LogLevel;

namespace {

void say(const dashcam::log::LogCallback& log, LvL lvl, const std::string& msg) {
    if (log) log(lvl, msg);
}

constexpr int kExecNotFound = 127;  ///< runArgv() return when the binary is not on PATH.

// Run args[0] with the given argv, capturing stdout, waiting up to timeoutMs.
// No shell is involved, so arbitrary argument bytes (e.g. an SSID) are safe.
// Returns the child exit code (0 = success), 127 if the binary was not found,
// or -1 on spawn failure / timeout (the child is killed on timeout).
int runArgv(const std::vector<std::string>& args, std::string* out, int timeoutMs) {
    if (out) out->clear();
    if (args.empty()) return -1;

    int pipefd[2];
    if (::pipe(pipefd) != 0) return -1;

    pid_t pid = ::fork();
    if (pid < 0) { ::close(pipefd[0]); ::close(pipefd[1]); return -1; }

    if (pid == 0) {
        // ── child ──
        ::close(pipefd[0]);
        ::dup2(pipefd[1], STDOUT_FILENO);
        int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) { ::dup2(devnull, STDERR_FILENO); ::close(devnull); }
        ::close(pipefd[1]);

        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
        argv.push_back(nullptr);
        ::execvp(argv[0], argv.data());
        _exit(kExecNotFound);  // execvp only returns on failure
    }

    // ── parent ──
    ::close(pipefd[1]);
    std::string buf;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    char tmp[4096];
    bool killed = false;
    for (;;) {
        const int remain = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                               deadline - std::chrono::steady_clock::now()).count());
        if (remain <= 0) { ::kill(pid, SIGKILL); killed = true; break; }

        struct pollfd p{pipefd[0], POLLIN, 0};
        int pr = ::poll(&p, 1, remain);
        if (pr < 0) { if (errno == EINTR) continue; break; }
        if (pr == 0) { ::kill(pid, SIGKILL); killed = true; break; }  // timeout

        ssize_t n = ::read(pipefd[0], tmp, sizeof(tmp));
        if (n > 0)      buf.append(tmp, static_cast<size_t>(n));
        else if (n == 0) break;                          // child closed stdout
        else { if (errno == EINTR) continue; break; }
    }
    ::close(pipefd[0]);

    int status = 0;
    ::waitpid(pid, &status, 0);
    if (out) *out = buf;
    if (killed) return -1;
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -1;
}

// Strip trailing whitespace/newlines.
std::string rtrim(std::string s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' ||
                          s.back() == ' '  || s.back() == '\t'))
        s.pop_back();
    return s;
}

// nmcli -t escapes ':' and '\' with a backslash; undo that for a single field.
std::string unescapeNmcli(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\' && i + 1 < s.size()) { out.push_back(s[++i]); }
        else out.push_back(s[i]);
    }
    return out;
}

// True while nmcli reports the overall STATE as connected.
bool nmConnected() {
    std::string out;
    if (runArgv({"nmcli", "-t", "-f", "STATE", "general"}, &out, 4000) != 0) return false;
    return rtrim(out).rfind("connected", 0) == 0;  // "connected" / "connected (site only)"
}

// True when NetworkManager reports full internet connectivity.
bool nmFullConnectivity() {
    std::string out;
    if (runArgv({"nmcli", "networking", "connectivity"}, &out, 4000) != 0) return false;
    return rtrim(out) == "full";
}

} // namespace

std::string currentWifiSsid(const dashcam::log::LogCallback& log) {
    // Preferred: the SSID of the currently-active WiFi AP.
    std::string out;
    if (runArgv({"nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"}, &out, 4000) == 0) {
        std::istringstream ss(out);
        std::string line;
        while (std::getline(ss, line)) {
            if (line.rfind("yes:", 0) == 0)
                return unescapeNmcli(rtrim(line.substr(4)));  // "yes:<ssid>"
        }
    }
    // Fallback: the first saved WiFi profile's name (NM names it after the SSID).
    std::string out2;
    if (runArgv({"nmcli", "-t", "-f", "NAME,TYPE", "connection", "show"}, &out2, 4000) == 0) {
        std::istringstream ss(out2);
        std::string line;
        while (std::getline(ss, line)) {
            if (line.find(":802-11-wireless") != std::string::npos)
                return unescapeNmcli(rtrim(line.substr(0, line.find(':'))));
        }
    }
    say(log, LvL::DEBUG, "currentWifiSsid: none found (nmcli/NetworkManager unavailable?)");
    return "";
}

WifiStatus connectWifi(const WifiConnectConfig& cfg, const dashcam::log::LogCallback& log) {
    WifiStatus st;  // defaults to Offline

    if (!cfg.enabled) {
        st.detail = "WiFi auto-connect disabled in config";
        return st;
    }

    std::string target = cfg.ssid.empty() ? currentWifiSsid(log) : cfg.ssid;
    if (target.empty()) {
        st.detail = "no known WiFi SSID (nmcli/NetworkManager unavailable or no saved profile)";
        say(log, LvL::WARN, "WiFi: " + st.detail);
        return st;
    }

    // Non-destructive fast path: already associated to the target SSID.
    if (nmConnected() && currentWifiSsid(log) == target) {
        if (cfg.requireInternet && !nmFullConnectivity()) {
            st.detail = "associated to '" + target + "' but no internet";
            say(log, LvL::WARN, "WiFi: " + st.detail);
            return st;
        }
        st.state  = ConnectivityState::Online;
        st.ssid   = target;
        st.detail = "already connected to '" + target + "'";
        say(log, LvL::INFO, "WiFi: " + st.detail);
        return st;
    }

    // Issue the connect using the saved profile's stored credentials.  Bounded by
    // nmcli -w; the process wait is given a little extra headroom.
    say(log, LvL::INFO, "WiFi: connecting to '" + target + "' (up to " +
                        std::to_string(cfg.timeoutSec) + " s)");
    std::string out;
    int rc = runArgv({"nmcli", "-w", std::to_string(cfg.timeoutSec),
                      "device", "wifi", "connect", target},
                     &out, (cfg.timeoutSec + 5) * 1000);
    if (rc == kExecNotFound) {
        st.detail = "nmcli not found (NetworkManager access not configured in this container?)";
        say(log, LvL::WARN, "WiFi: " + st.detail);
        return st;
    }
    if (rc != 0) {
        st.detail = "connect to '" + target + "' failed (nmcli rc=" + std::to_string(rc) + ")";
        say(log, LvL::WARN, "WiFi: " + st.detail);
        return st;
    }

    if (!nmConnected()) {
        st.detail = "not connected after attempting '" + target + "'";
        say(log, LvL::WARN, "WiFi: " + st.detail);
        return st;
    }
    if (cfg.requireInternet && !nmFullConnectivity()) {
        st.detail = "connected to '" + target + "' but no internet";
        say(log, LvL::WARN, "WiFi: " + st.detail);
        return st;
    }

    st.state  = ConnectivityState::Online;
    st.ssid   = target;
    st.detail = "connected to '" + target + "'";
    say(log, LvL::INFO, "WiFi: " + st.detail);
    return st;
}

} // namespace dashcam::network
