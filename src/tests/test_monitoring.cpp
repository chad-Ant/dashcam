#include "../monitoring.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <limits>
#include <poll.h>

int main() {
    hostproto::Telemetry telemetry{};
    telemetry.speed = 42;
    telemetry.sigSource = static_cast<uint8_t>(hostproto::VehSigSource::CAN_SNIFF);
    telemetry.accel = std::numeric_limits<float>::quiet_NaN();
    const auto live = dashcam::monitor::snapshot(telemetry, true, 10, true, 100, 30, 1);
    assert(live.find("\"speed_kmh\":42") != std::string::npos);
    assert(live.find("\"speed_source\":\"CAN\"") != std::string::npos);
    assert(live.find("\"acceleration_ms2\":null") != std::string::npos);
    assert(live.find("\"driver_available\":false") != std::string::npos);
    const auto stale = dashcam::monitor::snapshot(telemetry, false, 3000, false, 0, -1, 2);
    assert(stale.find("\"speed_kmh\":null") != std::string::npos);
    assert(stale.find("\"fps\":null") != std::string::npos);
    telemetry.speed = std::numeric_limits<float>::infinity();
    assert(dashcam::monitor::snapshot(telemetry, true, 10, true, 100, 30, 3)
           .find("\"speed_kmh\":null") != std::string::npos);

    const std::string path = "/tmp/dashcam-monitor-test-" + std::to_string(::getpid()) + ".sock";
    dashcam::monitor::Publisher publisher;
    assert(publisher.open(path));
    assert(!publisher.offer(live)); // missing agent does not wait
    const int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    assert(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    assert(::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);
    assert(publisher.offer(live));
    char buffer[16384];
    const auto bytes = ::recv(receiver, buffer, sizeof(buffer), 0);
    assert(bytes == static_cast<ssize_t>(live.size()));
    assert(std::string(buffer, static_cast<size_t>(bytes)) == live);
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 10000; ++i) publisher.offer(live); // stopped reader fills queue
    assert(std::chrono::steady_clock::now() - start < std::chrono::seconds(2));
    ::close(receiver);
    assert(::unlink(path.c_str()) == 0);
    assert(!publisher.offer(live));
    std::puts("monitoring: serialization, stale values, missing/stalled agent and socket delivery passed");
}
