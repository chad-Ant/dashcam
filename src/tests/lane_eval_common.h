// lane_eval_common.h — shared pieces of the lane test harness binaries
// (lane_drive): a bounded CSV writer and clock helpers.
//
// CsvWriter: producers (any thread, including the inference thread through a
// result sink) only append to a bounded queue; a writer thread drains it,
// fflush()es every second and fdatasync()s every 5 s.  When the queue is full
// the OLDEST rows are dropped and counted (drops()) — producers never block.
// Empty cell = invalid / unknown; NaN is never written (fmtF).

#ifndef LANE_EVAL_COMMON_H
#define LANE_EVAL_COMMON_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace laneeval {

inline int64_t clockNs(clockid_t id) {
    timespec ts{};
    clock_gettime(id, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
inline int64_t monoNs() { return clockNs(CLOCK_MONOTONIC); }
inline int64_t realNs() { return clockNs(CLOCK_REALTIME); }
inline int64_t bootNs() { return clockNs(CLOCK_BOOTTIME); }

/// %.9g (round-trips float32); empty for NaN/Inf.
inline std::string fmtF(double v) {
    if (!std::isfinite(v)) return std::string();
    char b[32];
    std::snprintf(b, sizeof(b), "%.9g", v);
    return b;
}

class CsvWriter {
public:
    explicit CsvWriter(size_t capacity = 20000) : cap_(capacity) {}
    ~CsvWriter() { close(); }
    CsvWriter(const CsvWriter&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;

    /// @p append: keep an existing file's rows (a restarted instance of the
    /// same run); the header goes only into a new or empty file.
    bool open(const std::string& path, const std::string& header, bool append = false) {
        f_ = std::fopen(path.c_str(), append ? "a" : "w");
        if (!f_) return false;
        if (append) std::fseek(f_, 0, SEEK_END);
        if (!append || std::ftell(f_) == 0) std::fprintf(f_, "%s\n", header.c_str());
        run_ = true;
        th_ = std::thread(&CsvWriter::loop, this);
        return true;
    }
    /// Queue one row (no trailing newline).  Never blocks on I/O.
    void row(std::string line) {
        {
            std::lock_guard<std::mutex> lk(m_);
            if (!run_) return;
            if (q_.size() >= cap_) { q_.pop_front(); ++drops_; }
            q_.push_back(std::move(line));
            ++rows_;
        }
        cv_.notify_one();
    }
    uint64_t drops() const { return drops_.load(); }
    uint64_t rows() const { return rows_.load(); }
    bool isOpen() const { return f_ != nullptr; }

    /// Drain, flush, fdatasync and close.  Idempotent.
    void close() {
        {
            std::lock_guard<std::mutex> lk(m_);
            if (!run_) return;
            run_ = false;
        }
        cv_.notify_one();
        if (th_.joinable()) th_.join();
        if (f_) {
            std::fflush(f_);
            fdatasync(fileno(f_));
            std::fclose(f_);
            f_ = nullptr;
        }
    }

private:
    void loop() {
        auto lastSync = std::chrono::steady_clock::now();
        std::deque<std::string> batch;
        for (;;) {
            bool running;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait_for(lk, std::chrono::seconds(1), [this] { return !q_.empty() || !run_; });
                batch.swap(q_);
                running = run_;
            }
            for (const auto& l : batch) {
                std::fputs(l.c_str(), f_);
                std::fputc('\n', f_);
            }
            batch.clear();
            std::fflush(f_);
            const auto now = std::chrono::steady_clock::now();
            if (now - lastSync >= std::chrono::seconds(5)) {
                fdatasync(fileno(f_));
                lastSync = now;
            }
            if (!running) {
                std::lock_guard<std::mutex> lk(m_);
                if (q_.empty()) return;
            }
        }
    }

    size_t cap_;
    std::FILE* f_ = nullptr;
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<std::string> q_;
    bool run_ = false;
    std::thread th_;
    std::atomic<uint64_t> drops_{0}, rows_{0};
};

} // namespace laneeval

#endif // LANE_EVAL_COMMON_H
