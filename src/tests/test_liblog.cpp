// test_liblog.cpp — smoke test for the async file+console logger (liblog).
//
// With no argument, init() writes to a "logs" directory next to the executable —
// i.e. this build's bin/build_<ts>/logs — created by the build.  The test uses
// logDir() to find that directory, leaves the log file there for inspection, and
// verifies: pre-init callback falls back to stderr; init() creates log_<ts>.txt
// under <exe_dir>/logs; all four levels reach the file; the async sink flushes
// mid-run (flush_on(warn)); producers never block under log pressure
// (overrun_oldest); post-shutdown callbacks fall back to stderr; second init()
// is a no-op.  A child re-invocation (--child-level) validates
// DASHCAM_LOG_LEVEL filtering and auto-creation of a missing nested log dir —
// both need a fresh process because init() is once-per-process.

#include "liblog.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
using namespace dashcam::log;

static int fails = 0;
#define CHECK(c,m) do{ if(!(c)){std::cerr<<"  FAIL  "<<m<<"\n";++fails;} else std::cout<<"  PASS  "<<m<<"\n";}while(0)

static std::string slurp(const std::string& p) {
    std::ifstream f(p);
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

// Newest log_*.txt in dir — the one init() just created (and is writing to).
static std::string newestLog(const std::string& dir) {
    std::string file;
    fs::file_time_type newest{};
    if (!fs::exists(dir)) return file;
    for (const auto& e : fs::directory_iterator(dir)) {
        const std::string n = e.path().filename().string();
        if (n.rfind("log_", 0) != 0) continue;
        const auto t = fs::last_write_time(e);
        if (file.empty() || t > newest) { newest = t; file = e.path().string(); }
    }
    return file;
}

// Child mode: run with DASHCAM_LOG_LEVEL=error and a nested directory that does
// not exist yet.  Validates env-based level filtering (DEBUG/INFO suppressed,
// ERROR kept) and that init() auto-creates the missing directory.  Exit code =
// failure count, read by the parent's CHECK.
static int childLevelTest(const std::string& dir) {
    init(dir);
    auto log = getCallback();
    log(LogLevel::DEBUG, "child-debug-suppressed");
    log(LogLevel::INFO,  "child-info-suppressed");
    log(LogLevel::ERROR, "child-error-kept");
    shutdown();

    const std::string file = newestLog(dir);
    if (file.empty()) { std::cerr << "child: no log file created\n"; return 1; }
    const std::string body = slurp(file);
    int f = 0;
    if (body.find("child-error-kept") == std::string::npos) { std::cerr << "child: ERROR line missing\n"; ++f; }
    if (body.find("suppressed") != std::string::npos)       { std::cerr << "child: suppressed line leaked\n"; ++f; }
    return f;
}

// Make @p fd a pipe that is full and blocking, with the read end kept open
// and never read: a stalled log collector.
static bool fillPipeOnto(int fd) {
    int p[2];
    if (::pipe(p) != 0) return false;
    ::fcntl(p[1], F_SETFL, O_NONBLOCK);
    static const char junk[4096] = {};
    while (::write(p[1], junk, sizeof junk) > 0) {}
    ::fcntl(p[1], F_SETFL, 0);
    return ::dup2(p[1], fd) >= 0;
}

// Child: the log FILE cannot be created (its directory would live under a
// regular file).  init() must still install the async logger, console only,
// so producers never block — even with stdout and stderr both full pipes
// (a synchronous stderr fallback would block right here).
static int childConsoleFallback(const std::string& file) {
    init(file + "/logs");
    int f = logDir().empty() ? 0 : 1;              // no file sink
    if (!fillPipeOnto(STDOUT_FILENO) || !fillPipeOnto(STDERR_FILENO)) return 20;
    auto log = getCallback();
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 20000; ++i) log(LogLevel::ERROR, "console-only-" + std::to_string(i));
    if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(2)) f += 2;
    ::_exit(f);                                    // not shutdown(): it would flush into the full pipe
}

// Child: emergencyExit() with the synchronous stderr fallback (no init())
// stuck on a full stderr pipe must still exit, with the given code.
static int childEmergency() {
    if (!fillPipeOnto(STDERR_FILENO)) return 20;
    emergencyExit(getCallback(), "emergency-exit-test", 7);
}

// Run this binary with @p args; exit status, or -1 if it had to be killed
// after @p limitMs.  @p ms receives the run time.
static int runChild(const std::vector<std::string>& args, int limitMs, long& ms) {
    char exe[4096];
    const ssize_t n = ::readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n <= 0) return -2;
    exe[n] = '\0';
    const auto t0 = std::chrono::steady_clock::now();
    const pid_t pid = ::fork();
    if (pid == 0) {
        std::vector<char*> av{exe};
        for (const auto& a : args) av.push_back(const_cast<char*>(a.c_str()));
        av.push_back(nullptr);
        ::execv(exe, av.data());
        ::_exit(127);
    }
    int status = 0;
    for (;;) {
        if (::waitpid(pid, &status, WNOHANG) == pid) break;
        if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(limitMs)) {
            ::kill(pid, SIGKILL);
            ::waitpid(pid, &status, 0);
            ms = limitMs;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    return WIFEXITED(status) ? WEXITSTATUS(status) : -3;
}

int main(int argc, char* argv[]) {
    if (argc == 3 && std::string(argv[1]) == "--child-level")
        return childLevelTest(argv[2]);
    if (argc == 3 && std::string(argv[1]) == "--child-console-fallback")
        return childConsoleFallback(argv[2]);
    if (argc == 2 && std::string(argv[1]) == "--child-emergency")
        return childEmergency();

    std::cout << "--- pre-init callback (should fall back to stderr, not crash) ---\n";
    getCallback()(LogLevel::WARN, "pre-init-warning");

    std::cout << "\n--- init() (build-local logs) + log all four levels ---\n";
    init();   // no arg -> <exe_dir>/logs, e.g. bin/build_<ts>/logs
    const std::string dir = logDir();
    std::cout << "  logDir() -> " << dir << "\n";
    CHECK(!dir.empty(), "logDir() reports a resolved directory");
    CHECK(dir.size() >= 4 && dir.substr(dir.size() - 4) == "logs", "log dir ends in .../logs");

    auto log = getCallback();
    log(LogLevel::DEBUG, "debug-alpha");
    log(LogLevel::INFO,  "info-bravo");
    log(LogLevel::WARN,  "warn-charlie");
    log(LogLevel::ERROR, "error-delta");

    // Let the async worker drain the queue, then find the file this run created.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const std::string file = newestLog(dir);
    CHECK(!file.empty(), "log_<ts>.txt created under the build's logs dir");
    const std::size_t bytesBefore = file.empty() ? 0 : slurp(file).size();
    // flush_on(warn) must push the buffered lines to disk mid-run (before shutdown).
    CHECK(bytesBefore > 0, "file flushed mid-run (flush_on(warn) installed)");

    // Producers must never block under log pressure: 4 threads x 5k messages
    // (20k total against an 8192-slot overrun_oldest queue).  The earlier
    // alpha..delta lines are already flushed to disk, so drops can't lose them.
    std::cout << "\n--- concurrent callback hammer (4 threads x 5k, overrun_oldest) ---\n";
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<std::thread> threads;
        for (int t = 0; t < 4; ++t) {
            threads.emplace_back([&log, t] {
                for (int i = 0; i < 5000; ++i)
                    log(LogLevel::DEBUG,
                        "hammer-" + std::to_string(t) + "-" + std::to_string(i));
            });
        }
        for (auto& th : threads) th.join();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        std::cout << "  20k messages produced in " << ms << " ms\n";
        CHECK(ms < 5000, "producers not blocked under log pressure (overrun_oldest)");
    }

    shutdown();   // flushes the async queue + file, closes the logger

    std::cout << "\n--- post-shutdown callback (stderr fallback, no crash) ---\n";
    getCallback()(LogLevel::INFO, "post-shutdown-info");
    CHECK(true, "post-shutdown callback fell back without crashing");

    const std::string body = file.empty() ? std::string() : slurp(file);
    std::cout << "  file bytes: before shutdown()=" << bytesBefore
              << "  after=" << body.size() << "\n";
    std::cout << "  --- log file head ---\n" << body.substr(0, 600)
              << "  --- end (" << body.size() << " bytes total) ---\n";

    CHECK(body.find("debug-alpha")  != std::string::npos, "DEBUG line present in file");
    CHECK(body.find("info-bravo")   != std::string::npos, "INFO line present in file");
    CHECK(body.find("warn-charlie") != std::string::npos, "WARN line present in file");
    CHECK(body.find("error-delta")  != std::string::npos, "ERROR line present in file");

    // init() is one-shot (std::call_once): a second call must not re-initialise.
    init("/tmp/liblog_should_not_appear");
    CHECK(!fs::exists("/tmp/liblog_should_not_appear"), "second init() is a no-op (call_once)");

    // DASHCAM_LOG_LEVEL filtering + nested-dir auto-creation need a fresh
    // process (init() is once-per-process), so re-invoke this binary in child
    // mode with the env var set and a directory that does not exist yet.
    std::cout << "\n--- child process: DASHCAM_LOG_LEVEL=error + nested dir auto-create ---\n";
    {
        const std::string childDir = "/tmp/liblog_test_child/nested/logs";
        std::error_code ec;
        fs::remove_all("/tmp/liblog_test_child", ec);

        char exe[4096];
        ssize_t n = ::readlink("/proc/self/exe", exe, sizeof(exe) - 1);
        if (n > 0) {
            exe[n] = '\0';
            const std::string cmd = "DASHCAM_LOG_LEVEL=error '" + std::string(exe) +
                                    "' --child-level " + childDir + " >/dev/null";
            const int rc = std::system(cmd.c_str());
            CHECK(rc == 0, "child: ERROR kept, DEBUG/INFO suppressed, nested dir auto-created");
        } else {
            CHECK(false, "readlink(/proc/self/exe) failed; child test not run");
        }
        fs::remove_all("/tmp/liblog_test_child", ec);
    }

    std::cout << "\n--- child processes: never block on a full pipe ---\n";
    {
        const std::string regular = "/tmp/liblog_test_regular_file";
        std::ofstream(regular) << "x";
        long ms = 0;
        const int rc = runChild({"--child-console-fallback", regular}, 10000, ms);
        CHECK(rc == 0, "log file impossible: async console-only logger, producers never block on a full "
                       "stdout (rc " + std::to_string(rc) + ", " + std::to_string(ms) + " ms)");
        std::error_code ec;
        fs::remove(regular, ec);
        const int rc2 = runChild({"--child-emergency"}, 10000, ms);
        CHECK(rc2 == 7 && ms < 2000, "emergencyExit exits (code 7) though the stderr fallback is stuck on a "
                                     "full pipe (rc " + std::to_string(rc2) + ", " + std::to_string(ms) + " ms)");
    }

    std::cout << "\nLog file left for inspection: " << file << "\n";
    std::cout << "=== " << fails << " failure(s) ===\n";
    return fails ? 1 : 0;
}
