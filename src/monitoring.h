#pragma once
// Best-effort local IPC only. Never opens a camera, writes a file, or waits for
// a reader. The host-side agent owns TLS, health collection and LAN clients.
#include "bridge_overlay.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <locale>
#include <sstream>

namespace dashcam::monitor {
inline double monotonicSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::string number(double n, bool valid = true) {
    if (!valid || !std::isfinite(n)) return "null";
    std::ostringstream s;
    s.imbue(std::locale::classic());
    s << std::setprecision(10) << n;
    return s.str();
}
inline std::string snapshot(const hostproto::Telemetry& t, bool fresh, int64_t ageMs,
                            bool recording, uint64_t frames, double fps, uint64_t seq) {
    record::OverlayData od;
    app::applyBridgeTelemetry(od, t, fresh, 0);
    const bool obd = fresh && (t.flags & hostproto::TLM_FLAG_OBD2_VALID);
    const bool canSpeed = fresh && hostproto::sigSourceSpeed(t.sigSource) == hostproto::VehSigSource::CAN_SNIFF;
    const char* speedSource = !od.speedValid ? "unavailable" : canSpeed ? "CAN" :
                             (t.fixValid && std::isfinite(t.gpsSpeedKmh)) ? "GNSS" : "OBD";
    std::ostringstream s;
    s.imbue(std::locale::classic());
    s << "{\"schema\":1,\"runtime\":\"dashcam_v0_4\",\"seq\":" << seq
      << ",\"source_mono_s\":" << std::setprecision(16) << monotonicSeconds()
      << ",\"recording\":" << (recording ? "true" : "false")
      << ",\"frames\":" << frames << ",\"fps\":" << number(fps, recording && fps >= 0)
      << ",\"bridge_fresh\":" << (fresh ? "true" : "false")
      << ",\"bridge_age_ms\":" << (ageMs < 0 ? "null" : std::to_string(ageMs))
      << ",\"vehicle\":{\"speed_kmh\":" << number(od.speedKmh, od.speedValid)
      << ",\"speed_source\":\"" << speedSource << "\",\"acceleration_ms2\":" << number(od.accelerationMs2, od.accelValid)
      << ",\"latitude\":" << number(od.latitude, od.positionValid)
      << ",\"longitude\":" << number(od.longitude, od.positionValid)
      << ",\"altitude_m\":" << number(od.altitudeM, od.positionValid)
      << ",\"heading_deg\":" << number(od.headingDeg, od.headingValid)
      << ",\"satellites\":" << (fresh ? std::to_string(t.satellites) : "null")
      << ",\"rpm\":" << number(t.rpm, fresh && (obd || hostproto::sigSourceRpm(t.sigSource) == hostproto::VehSigSource::CAN_SNIFF))
      << ",\"coolant_c\":" << number(t.coolantTemp, obd)
      << ",\"fuel_percent\":" << number(t.fuelLevel, obd)
      << ",\"throttle_percent\":" << number(t.throttle, obd)
      << ",\"engine_load_percent\":" << number(t.engineLoad, obd)
      << "},\"adas\":{\"lane_available\":false,\"driver_available\":false}}";
    return s.str();
}
class Publisher {
public:
    Publisher() = default;
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;
    ~Publisher() { if (fd_ >= 0) ::close(fd_); }
    bool open(const std::string& path) {
        if (fd_ >= 0 || path.empty() || path.size() >= sizeof(address_.sun_path)) return false;
        address_.sun_family = AF_UNIX;
        std::memcpy(address_.sun_path, path.c_str(), path.size() + 1);
        fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        return fd_ >= 0;
    }
    bool enabled() const { return fd_ >= 0; }
    bool offer(const std::string& json) {
        if (fd_ < 0 || json.size() > 16384) return false;
        return ::sendto(fd_, json.data(), json.size(), MSG_DONTWAIT | MSG_NOSIGNAL,
                        reinterpret_cast<const sockaddr*>(&address_), sizeof(address_)) == static_cast<ssize_t>(json.size());
    }
private:
    int fd_ = -1;
    sockaddr_un address_{};
};
} // namespace dashcam::monitor
