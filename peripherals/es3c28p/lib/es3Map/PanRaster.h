#pragma once
#include "Es3MapProtocol.h"

namespace es3map {
// Portable strip compositor. A draw pass freezes one pose/crop and two image
// leases, so a new pose or background cannot shear a half-drawn map.
class PanRaster {
public:
    PanView view;
    MapPose pose;

    void prepare(const PanView& v, const MapPose& p) {
        view = v; pose = p;
        const int32_t sine = sinQ10(p.courseCdeg);
        const int32_t cosine = sinQ10((uint32_t(p.courseCdeg) + 9000) % 36000);
        const int16_t halo[3][2] = {{0,224}, {-160,-160}, {160,-160}};
        const int16_t body[3][2] = {{0,176}, {-120,-128}, {120,-128}};
        for (int i = 0; i < 3; ++i) {
            rotate(halo[i], _halo[i], sine, cosine);
            rotate(body[i], _body[i], sine, cosine);
        }
    }
    bool sameView(const PanView& v, const MapPose& p) const {
        return view.cropX == v.cropX && view.cropY == v.cropY &&
            view.markerXQ4 == v.markerXQ4 && view.markerYQ4 == v.markerYQ4 &&
            view.markerVisible == v.markerVisible && pose.marker == p.marker &&
            (p.marker != MarkerArrow || pose.courseCdeg == p.courseCdeg);
    }
    bool rows(uint16_t* out, unsigned firstRow, unsigned count, const uint16_t* background,
              unsigned bgW, unsigned bgH, unsigned mapW, unsigned mapH) const {
        if (!out || !background || !count || count > 8 || !mapW || mapW > 320 ||
            firstRow > mapH || count > mapH - firstRow || view.cropX < 0 || view.cropY < 0 ||
            uint32_t(view.cropX) + mapW > bgW || uint32_t(view.cropY) + mapH > bgH) return false;
        for (unsigned row = 0; row < count; ++row) {
            const auto* src = background + (view.cropY + firstRow + row) * bgW + view.cropX;
            auto* dst = out + row * mapW;
            memcpy(dst, src, mapW * sizeof(uint16_t));
            if (view.markerVisible) {
                const int left = view.markerXQ4 / 16 - 17, right = view.markerXQ4 / 16 + 18;
                const unsigned begin = left > 0 ? unsigned(left) : 0;
                const unsigned end = right < int(mapW) ? unsigned(right) : mapW;
                for (unsigned x = begin; x < end; ++x) markerPixel(x, firstRow + row, dst[x]);
            }
        }
        return true;
    }
private:
    int32_t _halo[3][2]{}, _body[3][2]{};
    static int32_t sinQ10(uint32_t angle) {
        static const int16_t table[] = {0,89,178,265,350,433,512,587,658,724,784,839,887,928,962,989,1008,1020,1024};
        angle %= 36000;
        const bool negative = angle >= 18000;
        angle %= 18000;
        if (angle > 9000) angle = 18000 - angle;
        const unsigned i = angle / 500, f = angle % 500;
        const int32_t value = i == 18 ? table[18] : table[i] + (table[i+1] - table[i]) * f / 500;
        return negative ? -value : value;
    }
    static void rotate(const int16_t* in, int32_t* out, int32_t s, int32_t c) {
        out[0] = (in[0] * c + in[1] * s) / 1024;
        out[1] = (in[0] * s - in[1] * c) / 1024;
    }
    static bool triangle(int32_t x, int32_t y, const int32_t (&p)[3][2]) {
        bool pos = false, neg = false;
        for (int i = 0; i < 3; ++i) {
            const int j = (i + 1) % 3;
            const int64_t d = int64_t(x - p[i][0]) * (p[j][1] - p[i][1]) -
                              int64_t(y - p[i][1]) * (p[j][0] - p[i][0]);
            pos |= d > 0; neg |= d < 0;
        }
        return !(pos && neg);
    }
    void markerPixel(unsigned x, unsigned y, uint16_t& pixel) const {
        if (!view.markerVisible) return;
        const int64_t dx64 = int64_t(x) * 16 + 8 - view.markerXQ4;
        const int64_t dy64 = int64_t(y) * 16 + 8 - view.markerYQ4;
        if (dx64 < -256 || dx64 > 256 || dy64 < -256 || dy64 > 256) return;
        const int32_t dx = int32_t(dx64), dy = int32_t(dy64), radius2 = dx * dx + dy * dy;
        constexpr uint16_t white = 0xffff, blue = 0x133f, grey = 0x7bcf;
        if (pose.marker == MarkerArrow) {
            if (triangle(dx, dy, _halo)) pixel = white;
            if (triangle(dx, dy, _body)) pixel = blue;
        } else if (pose.marker == MarkerDot || pose.marker == MarkerLastKnown) {
            if (radius2 <= 136 * 136) pixel = white;
            if (pose.marker == MarkerDot && radius2 <= 96 * 96) pixel = blue;
            if (pose.marker == MarkerLastKnown && radius2 <= 132 * 132 && radius2 >= 92 * 92) pixel = grey;
        }
    }
};
} // namespace es3map
