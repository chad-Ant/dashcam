#include "liblog.h"

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cctype>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <unistd.h>
#include <vector>

namespace dashcam::log {

namespace {

// Accessed via std::atomic_load/store only, so the log callback can read it on
// any thread while init()/shutdown() publish or clear it without a data race.
std::shared_ptr<spdlog::logger> g_logger;
std::once_flag                  g_once;

// Resolved log directory.  Written once inside init()'s call_once, but read
// by logDir() from arbitrary threads — call_once only synchronises callers of
// call_once, so a bare std::string would race a logDir() call that overlaps
// init().  Guard both sides with a mutex (not a hot path).
std::mutex                      g_dirMutex;
std::string                     g_logDir;

// "logs" directory next to the running executable (independent of the cwd), e.g.
// the bin/build_<ts>/logs of the build this binary came from.  Falls back to a
// cwd-relative "logs" if the executable path can't be read.
std::string exeRelativeLogDir() {
    char buf[PATH_MAX];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "logs";
    buf[n] = '\0';
    std::string exe(buf);
    std::string::size_type slash = exe.find_last_of('/');
    std::string base = (slash == std::string::npos) ? std::string(".") : exe.substr(0, slash);
    return base + "/logs";
}

spdlog::level::level_enum toSpdLevel(LogLevel lvl) {
    switch (lvl) {
        case LogLevel::DEBUG: return spdlog::level::debug;
        case LogLevel::INFO:  return spdlog::level::info;
        case LogLevel::WARN:  return spdlog::level::warn;
        case LogLevel::ERROR: return spdlog::level::err;
    }
    return spdlog::level::info;
}

// Parse a level name (debug|info|warn|error|off, case-insensitive) with a
// fallback for unrecognised strings.
spdlog::level::level_enum levelFromString(std::string s, spdlog::level::level_enum fallback) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (s == "debug")                   return spdlog::level::debug;
    if (s == "info")                    return spdlog::level::info;
    if (s == "warn" || s == "warning")  return spdlog::level::warn;
    if (s == "error" || s == "err")     return spdlog::level::err;
    if (s == "off")                     return spdlog::level::off;
    return fallback;
}

// Initial level: LogParams::level, overridden by DASHCAM_LOG_LEVEL if set — a
// field unit can go verbose without editing the config file.
spdlog::level::level_enum initialLevel(const LogParams& params) {
    const spdlog::level::level_enum fromParams =
        levelFromString(params.level, spdlog::level::debug);
    const char* env = std::getenv("DASHCAM_LOG_LEVEL");
    if (!env) return fromParams;
    return levelFromString(env, fromParams);
}

} // namespace

void init(const std::string& logDir, const LogParams& params) {
    std::call_once(g_once, [&logDir, &params]() {

    // Empty logDir -> a "logs" dir next to the executable (build-local logs).
    const std::string dir = logDir.empty() ? exeRelativeLogDir() : logDir;

    // Local time so the filename matches spdlog's local-time line timestamps.
    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    struct tm tm{};
    localtime_r(&tt, &tm);

    char fname[PATH_MAX];
    int need = std::snprintf(fname, sizeof(fname), "%s/log_%04d%02d%02d_%02d%02d%02d.txt",
        dir.c_str(),
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);
    if (need < 0 || need >= static_cast<int>(sizeof(fname))) {
        // A silently truncated path would create the log somewhere unintended;
        // refuse instead — callbacks fall back to stderr (g_logger stays null).
        std::fprintf(stderr, "liblog: log path exceeds PATH_MAX, logging to stderr\n");
        return;
    }

    try {
        // Worker count stays 1 regardless of params: more workers would let
        // spdlog interleave lines out of order.  Guard degenerate tunables so
        // a hostile/corrupt config cannot zero the queue or the rotation.
        const size_t queueSize = params.queueSize > 0 ? params.queueSize : 8192;
        const size_t rotateKb  = params.rotateSizeKb > 0 ? params.rotateSizeKb : 3072;
        const size_t rotFiles  = params.rotateFiles > 0 ? params.rotateFiles : 3;
        spdlog::init_thread_pool(queueSize, 1);

        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            fname, rotateKb * 1024, rotFiles);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
        // overrun_oldest (not block): under log pressure drop the oldest queued
        // messages instead of blocking producers — a slow console must never
        // stall a capture/inference thread on this real-time box.
        auto logger = std::make_shared<spdlog::async_logger>(
            "dashcam",
            sinks.begin(), sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::overrun_oldest);
        logger->set_level(initialLevel(params));
        // Without a flush policy the async sinks buffer until shutdown(), so a
        // running app (or one killed before a clean shutdown) leaves the log file
        // empty.  Flush at params.flushOn (default WARN) immediately, and
        // everything else on a periodic timer so `tail -f` shows near-real-time
        // output without flushing every line (which would defeat the async
        // design on this real-time box).
        logger->flush_on(levelFromString(params.flushOn, spdlog::level::warn));
        spdlog::register_logger(logger);
        if (params.flushEverySec > 0)
            spdlog::flush_every(std::chrono::seconds(params.flushEverySec));
        {
            std::lock_guard<std::mutex> lk(g_dirMutex);
            g_logDir = dir;   // publish the resolved directory for logDir()
        }
        // Publish atomically; the callback reads it via std::atomic_load.
        // g_logger is shared_ptr<spdlog::logger>; upcast the async_logger so the
        // std::atomic_store overload deduces a single element type.
        std::atomic_store(&g_logger,
                          std::static_pointer_cast<spdlog::logger>(logger));
    } catch (const spdlog::spdlog_ex& ex) {
        std::fprintf(stderr, "liblog: spdlog init failed: %s\n", ex.what());
        // g_logger was never published; callbacks fall back to stderr.
    }

    }); // call_once
}

void shutdown() {
    // CONTRACT: all threads that call the log callback must be stopped first —
    // tearing down the async worker while another thread logs is unsafe.
    auto logger = std::atomic_load(&g_logger);
    if (!logger) return;
    logger->flush();
    // Clear the shared pointer first so any late callback falls back to stderr
    // instead of racing spdlog::shutdown().
    std::atomic_store(&g_logger, std::shared_ptr<spdlog::logger>{});
    spdlog::drop("dashcam");
    spdlog::shutdown();
}

std::string logDir() {
    std::lock_guard<std::mutex> lk(g_dirMutex);
    return g_logDir;
}

LogCallback getCallback() {
    return [](LogLevel lvl, const std::string& msg) {
        // Load once into a local: safe against a concurrent shutdown() reset,
        // and no TOCTOU between the null-check and the log call.
        auto logger = std::atomic_load(&g_logger);
        if (!logger) {
            std::fprintf(stderr, "[%s] %s\n",
                lvl == LogLevel::ERROR ? "ERROR" :
                lvl == LogLevel::WARN  ? "WARN"  :
                lvl == LogLevel::INFO  ? "INFO"  : "DEBUG",
                msg.c_str());
            return;
        }
        logger->log(toSpdLevel(lvl), msg);
    };
}

} // namespace dashcam::log
