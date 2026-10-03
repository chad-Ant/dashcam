#pragma once
/**
 * @file telemetry_log.h
 * @brief Every ESP32-C3 bridge frame to CSV: the decoded vehicle parameters at full rate.
 *
 * The ASS sidecar carries telemetry at SubtitleRateHz and only while recording;
 * this keeps every MSG_TELEMETRY (~10 Hz) and MSG_STATUS (1 Hz) frame the
 * bridge delivers, in two files per program start in the log directory:
 *
 *   telemetry_<YYYYMMDD_HHMMSS>.csv      one row per telemetry frame
 *   bridge_status_<YYYYMMDD_HHMMSS>.csv  one row per bridge status frame
 *
 * Each row starts with the host's epoch ms at arrival, so a file named under an
 * unset boot clock is still placed in time by its rows. Floats the master sent
 * as NaN (absent, stale, unavailable in this CAN mode) are empty cells; the
 * decoded vehicle bits are empty when their validity bit is clear, never "0".
 *
 * push*() runs on CommLink's RX thread, so it only copies the frame into a
 * bounded queue; a writer thread formats and writes, flushes every second and
 * fdatasync()s every five. A disk that stops answering costs queued rows
 * (oldest dropped first, counted), never the link.
 *
 * Enabled by <Log><TelemetryCsv>. Rows are written in arrival order.
 */

#include "HostProtocol.h"
#include "liblog.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

namespace dashcam {
namespace app {

namespace csv_fmt {
inline void add(std::string& row, const char* f, double v) {
    char b[48];
    std::snprintf(b, sizeof(b), f, v);
    row += ',';
    row += b;
}
/// A float cell: empty for NaN / non-finite.
inline void flt(std::string& row, float v, const char* f = "%.4g") {
    if (std::isfinite(v)) add(row, f, static_cast<double>(v));
    else row += ',';
}
inline void uns(std::string& row, unsigned long v) { row += ','; row += std::to_string(v); }
inline void hex(std::string& row, unsigned v, int digits) {
    char b[16];
    std::snprintf(b, sizeof(b), "0x%0*X", digits, v);
    row += ',';
    row += b;
}
/// A decoded bit, or an empty cell when @p valid is false.
inline void bit(std::string& row, bool valid, bool v) { row += valid ? (v ? ",1" : ",0") : ","; }
} // namespace csv_fmt

inline const char* telemetryCsvHeader() {
    return "host_ms,master_ms,speed_kmh,accel_ms2,rpm,coolant_c,fuel_level_pct,fuel_rate_lph,throttle_pct,"
           "engine_load_pct,baro_kpa,gear_cmd,gear_ratio,odo_km,lat,lon,alt_m,gps_speed_kmh,heading_deg,"
           "sats,fix_type,fix_valid,utc,imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,imu_mx,imu_my,imu_mz,"
           "imu_temp_c,imu_acc_peak,imu_gyro_peak,imu_lin_peak,imu_lin_x,imu_lin_y,imu_lin_z,highg_age_ms,"
           "highg_count,imu_yaw_rel_deg,imu_calib,canmap_checksum,canmap_flags,flags,can_mode,sig_source,"
           "src_speed,src_rpm,src_gear,src_steer,gear_pos,veh_flags,brake_pressed,brake_switch,turn_left,"
           "turn_right,hazard,pedal_pct,steer_torque,yaw_rate_dps,wheel_fl_kmh,wheel_fr_kmh,wheel_rl_kmh,"
           "wheel_rr_kmh,switch_state,switch_changed";
}

/// One telemetry frame as a CSV row (no newline); see telemetryCsvHeader().
inline std::string telemetryCsvRow(const hostproto::Telemetry& t, int64_t hostMs) {
    using namespace csv_fmt;
    using namespace hostproto;
    std::string r = std::to_string(hostMs);
    r.reserve(640);
    uns(r, t.masterMillis);
    flt(r, t.speed, "%.2f");   flt(r, t.accel, "%.3f");   flt(r, t.rpm, "%.0f");
    flt(r, t.coolantTemp);     flt(r, t.fuelLevel);       flt(r, t.fuelRate);
    flt(r, t.throttle);        flt(r, t.engineLoad);      flt(r, t.airPressure);
    flt(r, t.gear);            flt(r, t.gearRatio);       flt(r, t.odo, "%.1f");
    flt(r, t.latitude, "%.7f"); flt(r, t.longitude, "%.7f"); flt(r, t.altitude, "%.1f");
    flt(r, t.gpsSpeedKmh, "%.2f"); flt(r, t.heading, "%.1f");
    uns(r, t.satellites);      uns(r, t.fixType);         uns(r, t.fixValid);
    if (t.flags & TLM_FLAG_TIME_VALID) {
        char b[32];
        std::snprintf(b, sizeof(b), ",%04u-%02u-%02uT%02u:%02u:%02uZ", (unsigned)t.year, (unsigned)t.month,
                      (unsigned)t.day, (unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second);
        r += b;
    } else {
        r += ',';
    }
    flt(r, t.imuAccelX); flt(r, t.imuAccelY); flt(r, t.imuAccelZ);
    flt(r, t.imuGyroX);  flt(r, t.imuGyroY);  flt(r, t.imuGyroZ);
    flt(r, t.imuMagX);   flt(r, t.imuMagY);   flt(r, t.imuMagZ);
    flt(r, t.imuTempC, "%.1f");
    flt(r, t.imuAccelPeak); flt(r, t.imuGyroPeak); flt(r, t.imuLinAccelPeak);
    flt(r, t.imuLinAccelX); flt(r, t.imuLinAccelY); flt(r, t.imuLinAccelZ);
    if (t.imuHighGMs == 0xFFFFu) r += ','; else uns(r, t.imuHighGMs);
    uns(r, t.imuHighGCount);
    flt(r, t.imuYawRelDeg, "%.2f");
    hex(r, t.imuCalib, 2);  hex(r, t.canMapChecksum, 2);  hex(r, t.canMapFlags, 2);
    hex(r, t.flags, 4);     uns(r, t.canMode);            hex(r, t.sigSource, 2);
    uns(r, static_cast<unsigned>(sigSourceSpeed(t.sigSource)));
    uns(r, static_cast<unsigned>(sigSourceRpm(t.sigSource)));
    uns(r, static_cast<unsigned>(sigSourceGear(t.sigSource)));
    uns(r, static_cast<unsigned>(sigSourceSteer(t.sigSource)));
    uns(r, t.gearPos);
    const uint8_t vf = t.vehFlags;
    hex(r, vf, 2);
    const bool brakeOk = vf & VEH_FLAG_BRAKE_VALID, turnOk = vf & VEH_FLAG_TURN_VALID;
    bit(r, brakeOk, vf & VEH_FLAG_BRAKE_PRESSED);
    bit(r, brakeOk, vf & VEH_FLAG_BRAKE_SWITCH);
    bit(r, turnOk, vf & VEH_FLAG_TURN_LEFT);
    bit(r, turnOk, vf & VEH_FLAG_TURN_RIGHT);
    bit(r, turnOk, vf & VEH_FLAG_HAZARD);
    if (vf & VEH_FLAG_PEDAL_VALID) add(r, "%.1f", t.pedalGas * 0.5); else r += ',';
    if (t.steerMotorTorque == 0xFFFFu) r += ','; else uns(r, t.steerMotorTorque);
    if (t.yawRateCdps == INT16_MIN) r += ','; else add(r, "%.2f", t.yawRateCdps / 100.0);
    for (int i = 0; i < 4; ++i) {
        const uint16_t w = t.wheelRaw[i];
        if (w == 0xFFFFu) r += ','; else add(r, "%.2f", w * 0.01);
    }
    if (t.flags & TLM_FLAG_SWITCHES_PRESENT) hex(r, t.switchState, 4); else r += ',';
    hex(r, t.switchChanged, 4);
    return r;
}

inline const char* statusCsvHeader() {
    return "host_ms,bridge_ms,telemetry_age_ms,master_frames,master_crc_errors,host_frames,host_tx_dropped,"
           "battery_v,battery_pct,temp_c,free_heap_kb,battery_status,charge_state,flags,decimation";
}

/// One bridge status frame as a CSV row (no newline).
inline std::string statusCsvRow(const hostproto::BridgeStatus& s, int64_t hostMs) {
    using namespace csv_fmt;
    std::string r = std::to_string(hostMs);
    uns(r, s.bridgeMillis);
    if (s.telemetryAgeMs == UINT32_MAX) r += ','; else uns(r, s.telemetryAgeMs);
    uns(r, s.masterFrames); uns(r, s.masterCrcErrors); uns(r, s.hostFrames); uns(r, s.hostTxDropped);
    flt(r, s.batteryVolts, "%.3f"); flt(r, s.batteryPercent, "%.1f"); flt(r, s.tempC, "%.1f");
    uns(r, s.freeHeapKb); uns(r, s.batteryStatus); uns(r, s.chargeState);
    hex(r, s.flags, 2); uns(r, s.decimation);
    return r;
}

/**
 * @brief Writes bridge frames to the two CSV files (see the file comment).
 *
 * open() before the bridge's RX thread starts; close() after it stopped (it
 * joins the writer and flushes). push*() is safe from any thread.
 */
class TelemetryCsvLog {
public:
    static constexpr size_t kMaxQueued = 6000;   ///< ~10 min of frames at 10 Hz.

    TelemetryCsvLog() = default;
    ~TelemetryCsvLog() { close(); }
    TelemetryCsvLog(const TelemetryCsvLog&)            = delete;
    TelemetryCsvLog& operator=(const TelemetryCsvLog&) = delete;

    /// Creates both files in @p dir, named with the local time now. false (and
    /// a logged reason) when either cannot be created.
    bool open(const std::string& dir, dashcam::log::LogCallback log) {
        log_ = std::move(log);
        const std::time_t now = std::time(nullptr);
        std::tm tmv{};
        localtime_r(&now, &tmv);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmv);
        tlmPath_ = dir + "/telemetry_" + stamp + ".csv";
        stPath_  = dir + "/bridge_status_" + stamp + ".csv";
        tlm_ = std::fopen(tlmPath_.c_str(), "a");
        st_  = std::fopen(stPath_.c_str(), "a");
        if (!tlm_ || !st_) {
            say(dashcam::log::LogLevel::WARN, "telemetry CSV: cannot create " + (tlm_ ? stPath_ : tlmPath_) +
                                              " — bridge frames are not logged");
            closeFiles();
            return false;
        }
        std::fprintf(tlm_, "%s\n", telemetryCsvHeader());
        std::fprintf(st_, "%s\n", statusCsvHeader());
        std::fflush(tlm_);
        std::fflush(st_);
        stop_ = false;
        writer_ = std::thread([this] { run(); });
        open_.store(true);
        return true;
    }

    bool isOpen() const { return open_.load(); }
    const std::string& path() const { return tlmPath_; }
    uint64_t rowsWritten() const { return rows_.load(); }
    uint64_t dropped() const { return dropped_.load(); }

    void pushTelemetry(const hostproto::Telemetry& t, int64_t hostMs) {
        if (!open_.load()) return;
        std::lock_guard<std::mutex> lk(m_);
        if (tq_.size() >= kMaxQueued) { tq_.pop_front(); ++dropped_; }
        tq_.emplace_back(hostMs, t);
    }
    void pushStatus(const hostproto::BridgeStatus& s, int64_t hostMs) {
        if (!open_.load()) return;
        std::lock_guard<std::mutex> lk(m_);
        if (sq_.size() >= kMaxQueued) { sq_.pop_front(); ++dropped_; }
        sq_.emplace_back(hostMs, s);
    }

    /// Writes what is queued, joins the writer, closes both files. Safe twice.
    void close() {
        if (!open_.exchange(false)) return;
        {
            std::lock_guard<std::mutex> lk(m_);
            stop_ = true;
        }
        cv_.notify_all();
        if (writer_.joinable()) writer_.join();
        closeFiles();
    }

private:
    void say(dashcam::log::LogLevel lvl, const std::string& msg) { if (log_) log_(lvl, msg); }

    void closeFiles() {
        if (tlm_) { std::fflush(tlm_); ::fdatasync(fileno(tlm_)); std::fclose(tlm_); tlm_ = nullptr; }
        if (st_)  { std::fflush(st_);  ::fdatasync(fileno(st_));  std::fclose(st_);  st_  = nullptr; }
    }

    void run() {
        std::deque<std::pair<int64_t, hostproto::Telemetry>>    tq;
        std::deque<std::pair<int64_t, hostproto::BridgeStatus>> sq;
        auto nextSync = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        bool failed = false;
        uint64_t droppedSeen = 0;
        for (;;) {
            bool stopping;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait_for(lk, std::chrono::seconds(1), [this] { return stop_; });
                stopping = stop_;
                tq.swap(tq_);
                sq.swap(sq_);
            }
            bool ok = true;
            for (const auto& e : tq) {
                const std::string row = telemetryCsvRow(e.second, e.first);
                ok = std::fprintf(tlm_, "%s\n", row.c_str()) > 0 && ok;
            }
            for (const auto& e : sq) {
                const std::string row = statusCsvRow(e.second, e.first);
                ok = std::fprintf(st_, "%s\n", row.c_str()) > 0 && ok;
            }
            rows_ += tq.size() + sq.size();
            tq.clear();
            sq.clear();
            ok = std::fflush(tlm_) == 0 && ok;
            ok = std::fflush(st_) == 0 && ok;
            if (std::chrono::steady_clock::now() >= nextSync) {
                ::fdatasync(fileno(tlm_));
                ::fdatasync(fileno(st_));
                nextSync = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            }
            if (!ok && !failed) say(dashcam::log::LogLevel::WARN, "telemetry CSV: write to " + tlmPath_ + " failed");
            if (ok && failed)   say(dashcam::log::LogLevel::INFO, "telemetry CSV: writing again");
            failed = !ok;
            const uint64_t d = dropped_.load();
            if (d != droppedSeen) {
                say(dashcam::log::LogLevel::WARN, "telemetry CSV: " + std::to_string(d - droppedSeen) +
                                                  " frame(s) dropped (writer behind)");
                droppedSeen = d;
            }
            if (stopping) return;
        }
    }

    dashcam::log::LogCallback log_;
    std::string       tlmPath_, stPath_;
    std::FILE*        tlm_ = nullptr;
    std::FILE*        st_  = nullptr;
    std::thread       writer_;
    std::mutex        m_;
    std::condition_variable cv_;
    bool              stop_ = false;
    std::deque<std::pair<int64_t, hostproto::Telemetry>>    tq_;
    std::deque<std::pair<int64_t, hostproto::BridgeStatus>> sq_;
    std::atomic<bool>     open_{false};
    std::atomic<uint64_t> rows_{0};
    std::atomic<uint64_t> dropped_{0};
};

} // namespace app
} // namespace dashcam
