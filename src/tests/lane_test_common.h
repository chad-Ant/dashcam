// lane_test_common.h — shared test framework for lane_test (liblanedetector).
//
// House style (as camera_gst_test): check() prints "  ok  " / "  FAIL", a
// SIGALRM watchdog per test names the hung test, and the run ends with
// "RESULT: PASS|FAIL (N failures)".  On top of that, known-bug discipline:
//
//   bugCheck("B9", ok, what)  — a check of CORRECT behaviour that a listed,
//   not-yet-fixed bug is expected to break.  While the bug is in kKnownBugs a
//   failure prints "XFAIL B9" and does not count; a pass prints "XPASS B9"
//   and DOES count (the fix landed: remove B9 from kKnownBugs so the check
//   becomes a plain regression test).  Bugs not in the list are plain checks.
//
// --strict additionally demands that the set of bugs that XFAILed equals the
// --allow-xfail set exactly (with a test-name filter: is a subset of it).

#ifndef LANE_TEST_COMMON_H
#define LANE_TEST_COMMON_H

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <system_error>
#include <vector>
#include <unistd.h>

#include "liblanedetector.h"
#include "liblog.h"

namespace lanetest {

// Bugs of the lane test plan (~/drive_logs/lane_test_suite_plan_20261025.md,
// section 6) that are NOT fixed yet.  Remove an id when its fix lands.
// P4 (2026-10-25) fixed B1 B2 B3 (RateGate::kPts; the default stays kLegacy)
// B7 B9 B10 B12 B13 B14 B15 B17 B19.  B5 B6 B8 change results on valid input
// and are decided after the drives.
inline const std::set<std::string> kKnownBugs = {
    "B5", "B6", "B8",
};

inline int g_fails = 0;
inline std::set<std::string> g_xfailed;     // bugs that XFAILed this run
inline std::set<std::string> g_xpassed;     // bugs that XPASSed this run

inline void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

inline void bugCheck(const char* bug, bool ok, const std::string& what) {
    if (!kKnownBugs.count(bug)) {
        check(ok, std::string("[") + bug + "] " + what);
        return;
    }
    if (ok) {
        std::printf("  FAIL  XPASS %s (fixed? remove it from kKnownBugs): %s\n", bug, what.c_str());
        g_xpassed.insert(bug);
        ++g_fails;
    } else {
        std::printf("  xfail XFAIL %s: %s\n", bug, what.c_str());
        g_xfailed.insert(bug);
    }
}

/// Report-only measurement line (never fails).
inline void note(const std::string& what) { std::printf("  note  %s\n", what.c_str()); }

// ─── watchdog ────────────────────────────────────────────────────────────────

inline std::atomic<const char*>& stageName() {
    static std::atomic<const char*> s{"startup"};
    return s;
}

inline void onWatchdog(int) {
    const char* s = stageName().load();
    const char a[] = "\n  FAIL  watchdog: hung in \"";
    const char b[] = "\"\nRESULT: FAIL (hang)\n";
    ssize_t r = ::write(1, a, sizeof(a) - 1);
    r = ::write(1, s, std::strlen(s));
    r = ::write(1, b, sizeof(b) - 1);
    (void)r;
    _exit(2);
}

/// Names the running test for the watchdog and gives it @p budgetSec.
inline void stage(const char* name, unsigned budgetSec) {
    stageName() = name;
    ::alarm(budgetSec);
}

// ─── registry ────────────────────────────────────────────────────────────────

struct TestEnv {
    std::string engine;       ///< real UFLD v2 engine (GPU tier)
    std::string tinyEngine;   ///< constant-output tiny engine (GPU tier, optional)
    std::string fp16InEngine; ///< engine with an fp16 input binding (E2, optional)
    std::string fp16OutEngine;///< engine with fp16 output bindings (E2 control, optional)
};

struct TestCase {
    const char* name;
    unsigned    budgetSec;
    void      (*run)(const TestEnv&);
};

// lane_test live mode's pass rule (test_lanedetector.cpp), factored out so
// the unit tier can feed it synthetic poll sequences (plan bug B17).
struct LivePoll {
    bool     valid    = false;   ///< poll().valid
    uint64_t sequence = 0;       ///< poll().sequence
    bool     fresh    = false;   ///< hasFreshResult(500) at the poll
};
bool liveSmokeVerdict(const std::vector<LivePoll>& polls, bool cameraClosed, uint64_t processed);

// lane_test --video's core (test_lanedetector.cpp): every stride'th decoded
// frame of the first @p seconds (0 = all) through processFrame().  False on a
// pipeline error.  Exposed for the GPU tier's determinism test (plan B16).
bool runVideoResults(const std::string& engine, const std::string& video,
                     uint32_t w, uint32_t h, int seconds, int stride,
                     std::vector<dashcam::lane::LaneResult>& out);

// L12's child process (lane_test_gpu.cpp), run as "lane_test --child-b19 E".
int childB19(const std::string& engine);

std::vector<TestCase> unitTests();   // lane_test_unit.cpp — CPU only
std::vector<TestCase> gpuTests();    // lane_test_gpu.cpp  — CUDA / TensorRT / GStreamer

// ─── fixtures ────────────────────────────────────────────────────────────────

/// Collects everything a library logs, for checks on the diagnostics.
struct LogSink {
    std::mutex               m;
    std::vector<std::string> lines;
    dashcam::log::LogCallback callback() {
        return [this](dashcam::log::LogLevel lvl, const std::string& msg) {
            std::lock_guard<std::mutex> lk(m);
            lines.push_back(std::string(lvl == dashcam::log::LogLevel::ERROR ? "E " :
                                        lvl == dashcam::log::LogLevel::WARN  ? "W " : "I ") + msg);
        };
    }
    size_t count(const std::string& needle) {
        std::lock_guard<std::mutex> lk(m);
        size_t n = 0;
        for (const auto& l : lines) n += l.find(needle) != std::string::npos;
        return n;
    }
    bool has(const std::string& needle) { return count(needle) > 0; }
    void clear() {
        std::lock_guard<std::mutex> lk(m);
        lines.clear();
    }
};

// Data written by the suite must stay out of the public repo: true for any
// path inside it (realpath prefix check by path components against the repo
// this binary was built in and the two known checkout paths).  Fails closed:
// a path that cannot be resolved counts as inside.
inline bool insideRepo(const std::filesystem::path& p) {
    namespace fs = std::filesystem;
    std::error_code ec;
    // absolute() first: weakly_canonical leaves a relative path relative when
    // its first component does not exist yet.
    const fs::path abs = fs::absolute(p, ec);
    if (ec) return true;
    const fs::path real = fs::weakly_canonical(abs, ec);
    if (ec || real.empty() || !real.is_absolute()) return true;
    std::vector<fs::path> roots = { "/user/dashcam",
                                    "/home/jetson/Documents/github_repos/dashcam" };
    // The repo this binary was built in (bin/build_<ts>/lane_test), when it
    // really is one.
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        const fs::path root = exe.parent_path().parent_path().parent_path();
        if (root != root.root_path() && fs::exists(root / "Makefile", ec)
            && fs::exists(root / "lib" / "liblanedetector", ec))
            roots.push_back(root);
    }
    for (const auto& r : roots) {
        const fs::path rr = fs::weakly_canonical(r, ec);
        if (ec || rr.empty()) continue;
        auto ri = rr.begin();
        auto pi = real.begin();
        for (; ri != rr.end() && pi != real.end() && *ri == *pi; ++ri, ++pi) {}
        if (ri == rr.end()) return true;
    }
    return false;
}

// ─── runner ──────────────────────────────────────────────────────────────────

/// Runs the tests whose name contains @p filter; prints the RESULT line;
/// returns the exit code.  @p allowXfail is the --allow-xfail set.
inline int runTests(const char* tier, const std::vector<TestCase>& tests, const TestEnv& env,
                    const std::string& filter, bool strict,
                    const std::set<std::string>& allowXfail, bool listOnly) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);   // progress visible even if a test hangs
    std::signal(SIGALRM, onWatchdog);
    if (listOnly) {
        for (const auto& t : tests) std::printf("%s\n", t.name);
        return 0;
    }
    std::printf("lane_test %s tier\n", tier);
    int ran = 0;
    for (const TestCase& t : tests) {
        if (!filter.empty() && std::string(t.name).find(filter) == std::string::npos) continue;
        std::printf("\n[%s]\n", t.name);
        stage(t.name, t.budgetSec);
        t.run(env);
        ++ran;
    }
    ::alarm(0);
    if (ran == 0) {
        std::printf("no test matches \"%s\"\n", filter.c_str());
        return 1;
    }
    std::string xf;
    for (const auto& b : g_xfailed) xf += (xf.empty() ? "" : ",") + b;
    if (strict) {
        bool bad = false;
        for (const auto& b : g_xfailed)
            if (!allowXfail.count(b)) {
                std::printf("  FAIL  strict: %s XFAILed but is not in --allow-xfail\n", b.c_str());
                bad = true;
            }
        if (filter.empty())
            for (const auto& b : allowXfail)
                if (!g_xfailed.count(b)) {
                    std::printf("  FAIL  strict: --allow-xfail %s did not XFAIL\n", b.c_str());
                    bad = true;
                }
        if (bad) ++g_fails;
    }
    std::printf("\nRESULT: %s (%d failure%s, xfail [%s])\n", g_fails ? "FAIL" : "PASS",
                g_fails, g_fails == 1 ? "" : "s", xf.c_str());
    return g_fails ? 1 : 0;
}

} // namespace lanetest

#endif // LANE_TEST_COMMON_H
