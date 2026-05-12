#include "liblog.h"

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <memory>
#include <mutex>
#include <vector>

namespace dashcam::log {

namespace {

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

} // namespace

void init(const std::string& logDir) {
    std::call_once(g_once, [&logDir]() {

    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    struct tm tm{};
    gmtime_r(&tt, &tm);

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
        g_logger = std::make_shared<spdlog::async_logger>(
            "dashcam",
            sinks.begin(), sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::block);
        g_logger->set_level(spdlog::level::debug);
        spdlog::register_logger(g_logger);
    } catch (const spdlog::spdlog_ex& ex) {
        std::fprintf(stderr, "liblog: spdlog init failed: %s\n", ex.what());
        g_logger.reset();
    }

    }); // call_once
}

void shutdown() {
    if (!g_logger) return;
    g_logger->flush();
    spdlog::drop("dashcam");
    spdlog::shutdown();
    g_logger.reset();
}

LogCallback getCallback() {
    return [](LogLevel lvl, const std::string& msg) {
        if (!g_logger) {
            std::fprintf(stderr, "[%s] %s\n",
                lvl == LogLevel::ERROR ? "ERROR" :
                lvl == LogLevel::WARN  ? "WARN"  :
                lvl == LogLevel::INFO  ? "INFO"  : "DEBUG",
                msg.c_str());
            return;
        }
        g_logger->log(toSpdLevel(lvl), msg);
    };
}

} // namespace dashcam::log
