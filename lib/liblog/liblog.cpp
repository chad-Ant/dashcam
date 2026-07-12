#include "liblog.h"

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace dashcam::log {

namespace {

// Accessed via std::atomic_load/store only, so the log callback can read it on
// any thread while init()/shutdown() publish or clear it without a data race.
std::shared_ptr<spdlog::logger> g_logger;
std::once_flag                  g_once;

spdlog::level::level_enum toSpdLevel(LogLevel lvl) {
    switch (lvl) {
        case LogLevel::DEBUG: return spdlog::level::debug;
        case LogLevel::INFO:  return spdlog::level::info;
        case LogLevel::WARN:  return spdlog::level::warn;
        case LogLevel::ERROR: return spdlog::level::err;
    }
    return spdlog::level::info;
}

// Initial level from DASHCAM_LOG_LEVEL (debug|info|warn|error|off); default debug.
spdlog::level::level_enum levelFromEnv() {
    const char* env = std::getenv("DASHCAM_LOG_LEVEL");
    if (!env) return spdlog::level::debug;
    std::string s(env);
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (s == "info")                    return spdlog::level::info;
    if (s == "warn" || s == "warning")  return spdlog::level::warn;
    if (s == "error" || s == "err")     return spdlog::level::err;
    if (s == "off")                     return spdlog::level::off;
    return spdlog::level::debug;
}

} // namespace

void init(const std::string& logDir) {
    std::call_once(g_once, [&logDir]() {

    // Local time so the filename matches spdlog's local-time line timestamps.
    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    struct tm tm{};
    localtime_r(&tt, &tm);

    char fname[256];
    std::snprintf(fname, sizeof(fname), "%s/log_%04d%02d%02d_%02d%02d%02d.txt",
        logDir.c_str(),
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);

    try {
        spdlog::init_thread_pool(8192, 1);

        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            fname, 3 * 1024 * 1024, /*max_files=*/3);
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
        logger->set_level(levelFromEnv());
        spdlog::register_logger(logger);
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
