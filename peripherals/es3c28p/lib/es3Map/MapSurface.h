#pragma once
// Portable surface routing. All methods are called under Es3Video's mutex.
// Reuses the existing receiver and buffer leases; no second image pool.
#include "VideoState.h"
#include "Es3MapProtocol.h"

namespace es3map {
constexpr uint8_t kPageCount = 6;
constexpr uint8_t kMapPage = 5;
constexpr uint16_t kStatusHeight = 16;
constexpr uint16_t kAttributionTop = 208;
constexpr uint16_t kTabTop = 224;

inline int tabEdge(unsigned index, unsigned count) {
    return count ? int(index * 320 / count) : 0;
}
inline int tabAt(int x, unsigned count) {
    if (x < 0 || x >= 320 || !count || count > kMaxPages) return -1;
    for (unsigned i = 0; i < count; ++i)
        if (x < tabEdge(i + 1, count)) return int(i);
    return -1;
}
inline MapCapabilities capabilities(const es3video::Capabilities& video) {
    MapCapabilities m;
    m.pageCount = kPageCount; m.mapPage = kMapPage;
    m.viewportW = 320; m.viewportH = kTabTop;
    m.maxJpegBytes = video.maxJpegBytes; m.flags = kFlagMapTab;
    return m;
}
enum class Surface : uint8_t { Camera, Map };
struct Binding {
    Surface surface = Surface::Camera;
    uint32_t generation = 0;
    uint8_t page() const { return surface == Surface::Map ? kMapPage : 0; }
};

class SurfaceState : public es3video::VideoState {
public:
    Binding binding;
    void view(uint8_t page, bool blocked) {
        _page = page; _blocked = blocked;
        setDemand(currentDemand());
    }
    bool startCamera(uint32_t session, const es3video::StartVideoRequest& request,
                     const es3video::Capabilities& caps) {
        if (!session || !es3video::validStartVideo(request, caps) ||
            (token && binding.surface != Surface::Camera)) return false;
        binding = Binding();
        demand = currentDemand();
        start(session, request);
        return true;
    }
    bool startMap(uint32_t session, const StartMapRequest& request,
                  const es3video::Capabilities& caps) {
        if (!session || !validStartMap(request, capabilities(caps), caps) ||
            (token && binding.surface != Surface::Map)) return false;
        binding.surface = Surface::Map; binding.generation = request.generation;
        demand = currentDemand();
        start(session, sessionLimits(request));
        return true;
    }
    void stopSurface(Surface source, uint32_t session) {
        if (session && session == token && source == binding.surface) reset();
    }
private:
    uint8_t _page = 0;
    bool _blocked = true; // until graphics publishes its initial view
    es3video::Demand currentDemand() const {
        return _blocked ? es3video::DemandCalibrating :
            (_page == binding.page() ? es3video::DemandActive : es3video::DemandHidden);
    }
};
} // namespace es3map
