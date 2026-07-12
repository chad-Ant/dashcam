#ifndef LIBLOG_H
#define LIBLOG_H

#include <functional>
#include <string>

namespace dashcam::log {

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

using LogCallback = std::function<void(LogLevel, const std::string&)>;

// ─── Async file logger ────────────────────────────────────────────────────────
// Call init() once at application startup.  Creates log_<YYYYMMDD_HHMMSS>.txt
// (local time, matching the in-file line timestamps) in logDir, plus colored
// console output.  init() is one-shot for the process lifetime — it runs at most
// once (std::call_once), so a second init(), including after shutdown(), is a
// no-op.
//
// The default level is DEBUG; override it with the DASHCAM_LOG_LEVEL environment
// variable (debug|info|warn|error|off).
//
// getCallback() returns a LogCallback that is safe to call from any number of
// concurrent threads and is safe against a concurrent shutdown() (it falls back
// to stderr once the logger is torn down).  Under log pressure the async queue
// drops the oldest messages rather than blocking the caller, so a slow console
// never stalls a producer thread.
//
// shutdown() flushes and closes the log.  CONTRACT: stop/join every thread that
// may call the log callback before invoking shutdown() — tearing down the async
// worker while another thread is mid-log is inherently unsafe.
void        init(const std::string& logDir = ".");
void        shutdown();
LogCallback getCallback();

} // namespace dashcam::log

#endif // LIBLOG_H
