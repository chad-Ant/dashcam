#ifndef LIBLOG_H
#define LIBLOG_H

#include <functional>
#include <string>

namespace dashcam::log {

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

using LogCallback = std::function<void(LogLevel, const std::string&)>;

// ─── Async file logger ────────────────────────────────────────────────────────
// Call init() once at application startup.  Creates log_<YYYYMMDD_HHMMSS>.txt
// in logDir.  Call shutdown() at exit to flush and close the log file.
// getCallback() returns a thread-safe LogCallback that enqueues to the async
// worker; it is safe to call from any number of concurrent threads.
void        init(const std::string& logDir = ".");
void        shutdown();
LogCallback getCallback();

} // namespace dashcam::log

#endif // LIBLOG_H
