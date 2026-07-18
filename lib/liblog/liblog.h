#ifndef LIBLOG_H
#define LIBLOG_H

#include <functional>
#include <string>

namespace dashcam::log {

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

using LogCallback = std::function<void(LogLevel, const std::string&)>;

/**
 * @brief Tunable logger parameters.  Defaults preserve the historical
 *        hardcoded values.
 *
 * The application copies these from dashcam::config::LogConfig (liblog cannot
 * depend on libconfig — libconfig already depends on liblog) and passes them to
 * init().  The async worker count is deliberately NOT tunable: it is fixed at 1
 * so log lines keep their global order (multiple spdlog workers may interleave).
 */
struct LogParams {
    uint32_t    queueSize     = 8192;    ///< Async queue depth (messages); oldest dropped beyond this.
    uint32_t    rotateSizeKb  = 3072;    ///< Max log file size before rotation (KB).
    uint32_t    rotateFiles   = 3;       ///< Rotated files kept per log.
    uint32_t    flushEverySec = 1;       ///< Periodic flush interval (s); 0 disables the timer.
    std::string level         = "debug"; ///< Minimum level written (debug|info|warn|error|off).
                                         ///< The DASHCAM_LOG_LEVEL env var overrides this.
    std::string flushOn       = "warn";  ///< Level that forces an immediate flush (off = never).
};

// ─── Async file logger ────────────────────────────────────────────────────────
// Call init() once at application startup.  Creates log_<YYYYMMDD_HHMMSS>.txt
// (local time, matching the in-file line timestamps) in logDir, plus colored
// console output.  init() is one-shot for the process lifetime — it runs at most
// once (std::call_once), so a second init(), including after shutdown(), is a
// no-op.
//
// logDir defaults to EMPTY, which resolves to a "logs" directory next to the
// running executable (i.e. the bin/build_<ts>/logs of the build the binary came
// from — the build creates that directory too).  Resolution uses the executable
// path, so it is independent of the current working directory.  Pass an explicit
// path to override.  The directory is created if it does not exist.
//
// The minimum level comes from LogParams::level (default DEBUG); the
// DASHCAM_LOG_LEVEL environment variable (debug|info|warn|error|off) overrides
// it, so a field unit can be switched to verbose logging without editing the
// config file.
//
// getCallback() returns a LogCallback that is safe to call from any number of
// concurrent threads.  Once shutdown() has completed, late callbacks fall back
// to stderr.  Under log pressure the async queue drops the oldest messages
// rather than blocking the caller, so a slow console never stalls a producer
// thread.
//
// shutdown() flushes and closes the log.  CONTRACT: stop/join every thread that
// may call the log callback before invoking shutdown().  A callback racing
// shutdown() itself will not crash — the logger pointer is swapped atomically
// and spdlog absorbs a post to the dying worker — but that message may be
// lost; only the stop-threads-first ordering guarantees delivery.
void        init(const std::string& logDir = "", const LogParams& params = {});
void        shutdown();
LogCallback getCallback();

// The directory init() resolved the log file into.  Empty until init() runs (or
// if the file sink failed and logging fell back to stderr).
std::string logDir();

} // namespace dashcam::log

#endif // LIBLOG_H
