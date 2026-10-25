// lane_test_gpu.cpp — lane_test --gpu: CUDA / TensorRT / GStreamer tests of
// liblanedetector (T1/T2 of the lane test plan,
// ~/drive_logs/lane_test_suite_plan_20261025.md section 8).
//
// T1 "GPU component" (no live pipeline, no camera):
//   P1–P5  launchPreprocessKernel on cudaMalloc'd buffers vs a CPU bilinear
//          reference (align_corners=false: half-pixel centres, floor + border
//          clamp, BGR→RGB, ImageNet mean/std, HWC→CHW), in double precision
//   E1–E5  engine contract: IO via the TensorRT API, fp16 IO engines
//          rejected (B10 / control), load cost, bit-identical reruns, identity
//   PF1–PF3 processFrame determinism + timings, refusal while running,
//          setResultSink
//   T1–T2  tiny constant-output engine (tools/lane_eval/make_tiny_engine.py):
//          decode end to end on real TensorRT buffers, runModelSized
//
// T2 lifecycle / pipeline (appsrc-fed detector bins, no camera):
//   L1–L14 createBin while running (B1), spin after stop/EOS (B2), rate
//          gate (B3), restart state (B12), NVMM size mismatch (B7), padded
//          stride (B15), error flood (B14), setLogCallback while running
//          (B13), 50 restart cycles, golden row pattern, throwing log
//          callback in a child process (B19), timestamps, two detectors
//   S2     lane_test --video determinism (B16)
//
// Engines: --engine (real UFLD v2, required), --tiny / --fp16-in / --fp16-out
// (optional; a test needing an absent one notes "skipped").

#include "lane_test_common.h"
#include "lane_geometry.h"
#include "liblanedetector.h"
#include "liblanedetector_preprocess.h"
#include "ufld_decode.h"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferVersion.h>
#include <cuda_runtime.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace dashcam::lane;
using lanetest::LogSink;
using lanetest::TestCase;
using lanetest::TestEnv;
using lanetest::bugCheck;
using lanetest::check;
using lanetest::note;

namespace {

std::string f(double v, int prec = 3) {
    char b[48];
    std::snprintf(b, sizeof(b), "%.*f", prec, v);
    return b;
}

std::string g(double v) {
    char b[48];
    std::snprintf(b, sizeof(b), "%.3g", v);
    return b;
}

double nowMs() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool sameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

bool sameBits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() &&
           (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0);
}

bool sameSlot(const LaneSlot& a, const LaneSlot& b) {
    return a.detected == b.detected && a.head == b.head && a.nPts == b.nPts &&
           sameBits(a.slope, b.slope) && sameBits(a.intercept, b.intercept) &&
           sameBits(a.xRef, b.xRef) && sameBits(a.yMin, b.yMin) &&
           sameBits(a.yMax, b.yMax) && sameBits(a.rmsPx, b.rmsPx);
}

// Everything a result says about the road (not sequence / timing).
bool sameResult(const LaneResult& a, const LaneResult& b) {
    bool ok = a.valid == b.valid && a.numLanes == b.numLanes &&
              a.currentLaneIndex == b.currentLaneIndex &&
              sameBits(a.lateralOffset, b.lateralOffset) &&
              a.lateralValid == b.lateralValid && a.nonFinite == b.nonFinite;
    for (size_t i = 0; i < a.slots.size(); ++i) ok = ok && sameSlot(a.slots[i], b.slots[i]);
    return ok;
}

long memAvailableKb() {
    std::ifstream in("/proc/meminfo");
    std::string key;
    long v = -1;
    std::string unit;
    while (in >> key >> v >> unit)
        if (key == "MemAvailable:") return v;
    return -1;
}

bool fileExists(const std::string& p) {
    std::ifstream in(p, std::ios::binary);
    return in.good();
}

// ─── frames (BGR, HWC, uint8) ────────────────────────────────────────────────

struct Frame {
    int w = 0, h = 0;
    std::vector<uint8_t> px;   // w·h·3, B G R
    Frame(int w_, int h_, uint8_t fill = 0) : w(w_), h(h_), px(static_cast<size_t>(w_) * h_ * 3, fill) {}
    uint8_t* at(int x, int y) { return &px[(static_cast<size_t>(y) * w + x) * 3]; }
    const uint8_t* at(int x, int y) const { return &px[(static_cast<size_t>(y) * w + x) * 3]; }
};

Frame randomFrame(int w, int h, uint32_t seed) {
    Frame fr(w, h);
    std::mt19937 rng(seed);
    for (auto& b : fr.px) b = static_cast<uint8_t>(rng() & 0xFF);
    return fr;
}

// Hard 0/255 edges, a different phase per channel (every interpolation
// weight matters and channel mix-ups show).
Frame checkerFrame(int w, int h, int cell) {
    Frame fr(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            uint8_t* p = fr.at(x, y);
            p[0] = ((x / cell + y / cell) & 1) ? 255 : 0;
            p[1] = ((x / (cell + 1) + y / cell) & 1) ? 255 : 0;
            p[2] = ((x / cell + y / (cell + 3)) & 1) ? 0 : 255;
        }
    return fr;
}

Frame gradientFrame(int w, int h) {
    Frame fr(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            uint8_t* p = fr.at(x, y);
            p[0] = static_cast<uint8_t>((x * 255) / std::max(1, w - 1));            // B: horizontal
            p[1] = static_cast<uint8_t>((y * 255) / std::max(1, h - 1));            // G: vertical
            p[2] = static_cast<uint8_t>(((x + y) * 255) / std::max(1, w + h - 2));  // R: diagonal
        }
    return fr;
}

// A synthetic road for the real engine: dark asphalt with noise, two bright
// boundaries converging upwards.  Whatever the model makes of it, it is a
// fixed, non-trivial input.
Frame roadFrame(int w, int h, uint32_t seed) {
    Frame fr(w, h);
    std::mt19937 rng(seed);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const int v = 70 + static_cast<int>(rng() % 20);
            uint8_t* p = fr.at(x, y);
            p[0] = p[1] = p[2] = static_cast<uint8_t>(v);
        }
    for (int y = 0; y < h; ++y) {
        const double t = static_cast<double>(y) / (h - 1);   // 0 top .. 1 bottom
        const int xl = static_cast<int>(w * (0.42 - 0.30 * t));
        const int xr = static_cast<int>(w * (0.58 + 0.30 * t));
        const int half = 2 + static_cast<int>(6 * t);
        for (int dx = -half; dx <= half; ++dx) {
            for (int xc : { xl + dx, xr + dx })
                if (xc >= 0 && xc < w) {
                    uint8_t* p = fr.at(xc, y);
                    p[0] = p[1] = p[2] = 235;
                }
        }
    }
    return fr;
}

// ─── preprocess: GPU run and CPU reference ───────────────────────────────────

struct Norm {
    float meanR = 0.485f, meanG = 0.456f, meanB = 0.406f;
    float stdR  = 0.229f, stdG  = 0.224f, stdB  = 0.225f;
};

struct PreParams {
    int dstW = 1600, dstH = 320;
    float yOff = 0.0f, yScale = 1.0f;
    Norm n;
};

PreParams paramsFor(const LaneGeometry& geo) {
    PreParams p;
    p.yOff = geo.preYOff;
    p.yScale = geo.preYScale;
    return p;
}

struct DevMem {
    void* p = nullptr;
    explicit DevMem(size_t bytes) {
        if (cudaMalloc(&p, bytes) != cudaSuccess) p = nullptr;
    }
    ~DevMem() { cudaFree(p); }
    DevMem(const DevMem&) = delete;
    DevMem& operator=(const DevMem&) = delete;
};

// Runs the kernel on @p fr.  With @p guardBytes > 0 the frame sits between
// two guard bands of @p guardValue, so an out-of-bounds read shows in the
// output.  Returns an empty vector on any CUDA error (reported).
std::vector<float> gpuPre(const Frame& fr, const PreParams& p,
                          size_t guardBytes = 0, uint8_t guardValue = 0) {
    const size_t frameBytes = fr.px.size();
    const size_t outFloats = static_cast<size_t>(3) * p.dstW * p.dstH;
    DevMem src(frameBytes + 2 * guardBytes);
    DevMem dst(outFloats * sizeof(float));
    std::vector<float> out;
    if (!src.p || !dst.p) {
        check(false, "cudaMalloc for the preprocess test");
        return out;
    }
    uint8_t* s = static_cast<uint8_t*>(src.p);
    bool ok = true;
    if (guardBytes) {
        ok = ok && cudaMemset(s, guardValue, guardBytes) == cudaSuccess;
        ok = ok && cudaMemset(s + guardBytes + frameBytes, guardValue, guardBytes) == cudaSuccess;
    }
    ok = ok && cudaMemcpy(s + guardBytes, fr.px.data(), frameBytes, cudaMemcpyHostToDevice) == cudaSuccess;
    ok = ok && cudaMemset(dst.p, 0xFF, outFloats * sizeof(float)) == cudaSuccess;   // NaN fill: unwritten shows
    const cudaError_t launch = launchPreprocessKernel(
        s + guardBytes, static_cast<float*>(dst.p), fr.w, fr.h, p.dstW, p.dstH,
        p.yOff, p.yScale, p.n.meanR, p.n.meanG, p.n.meanB, p.n.stdR, p.n.stdG, p.n.stdB, nullptr);
    ok = ok && launch == cudaSuccess && cudaDeviceSynchronize() == cudaSuccess;
    if (ok) {
        out.resize(outFloats);
        ok = cudaMemcpy(out.data(), dst.p, outFloats * sizeof(float), cudaMemcpyDeviceToHost) == cudaSuccess;
    }
    if (!ok) {
        check(false, std::string("preprocess kernel run: ") + cudaGetErrorString(cudaGetLastError()));
        out.clear();
    }
    return out;
}

// The specification, independently of the kernel: align_corners=false
// (output centre (d+0.5)·scale − 0.5, vertical ROI yOff + (dy+0.5)·yScale −
// 0.5), floor + clamp at the borders, RGB planes, (v/255 − mean)/std; the
// whole computation in double.
std::vector<float> refPre(const Frame& fr, const PreParams& p) {
    const size_t plane = static_cast<size_t>(p.dstW) * p.dstH;
    std::vector<float> out(3 * plane);
    const double mean[3] = { p.n.meanR, p.n.meanG, p.n.meanB };
    const double stdv[3] = { p.n.stdR, p.n.stdG, p.n.stdB };
    auto clampi = [](int v, int hi) { return std::max(0, std::min(v, hi)); };
    for (int dy = 0; dy < p.dstH; ++dy) {
        const double sy = static_cast<double>(p.yOff) + (dy + 0.5) * static_cast<double>(p.yScale) - 0.5;
        const int iy = static_cast<int>(std::floor(sy));
        const double fy = sy - iy;
        const int y0 = clampi(iy, fr.h - 1), y1 = clampi(iy + 1, fr.h - 1);
        for (int dx = 0; dx < p.dstW; ++dx) {
            const double sx = (dx + 0.5) * (static_cast<double>(fr.w) / p.dstW) - 0.5;
            const int ix = static_cast<int>(std::floor(sx));
            const double fx = sx - ix;
            const int x0 = clampi(ix, fr.w - 1), x1 = clampi(ix + 1, fr.w - 1);
            for (int c = 0; c < 3; ++c) {
                const int bgr = 2 - c;   // plane 0 = R = byte 2
                const double v = (1 - fy) * ((1 - fx) * fr.at(x0, y0)[bgr] + fx * fr.at(x1, y0)[bgr])
                               +      fy  * ((1 - fx) * fr.at(x0, y1)[bgr] + fx * fr.at(x1, y1)[bgr]);
                out[c * plane + static_cast<size_t>(dy) * p.dstW + dx] =
                    static_cast<float>((v / 255.0 - mean[c]) / stdv[c]);
            }
        }
    }
    return out;
}

// max |a − b|; NaN anywhere (an unwritten output) counts as +inf.
double maxAbsDiff(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size() || a.empty()) return INFINITY;
    double m = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double d = std::fabs(static_cast<double>(a[i]) - b[i]);
        if (!(d == d)) return INFINITY;
        m = std::max(m, d);
    }
    return m;
}

double normValue(double byteVal, int plane, const Norm& n) {
    const double mean[3] = { n.meanR, n.meanG, n.meanB };
    const double stdv[3] = { n.stdR, n.stdG, n.stdB };
    return (byteVal / 255.0 - mean[plane]) / stdv[plane];
}

// Tolerance for kernel vs reference.  The kernel computes sample coordinates
// in float: near x = 1455 one float ulp is ~1.2e-4 px, which between
// neighbours 255 apart moves the output by ~0.03 grey levels — measured
// max |Δ| 4.3e-4 (normalised) on random 1456→1600 frames, 2.4e-7 at identity
// geometry.  The plan's 1e-4 is therefore unreachable by any float kernel;
// 2e-3 is ~0.12 of one input grey level (1/255/0.229 = 0.017), while every
// real defect (wrong pixel, channel, weight or ROI) moves values by whole
// grey levels.
constexpr double kPreTol = 2e-3;

void comparePre(const std::string& tag, const Frame& fr, const PreParams& p, double* worst = nullptr) {
    const std::vector<float> gpu = gpuPre(fr, p);
    const std::vector<float> ref = refPre(fr, p);
    const double d = maxAbsDiff(gpu, ref);
    if (worst) *worst = std::max(*worst, d);
    check(d <= kPreTol, tag + ": max |gpu - ref| " + g(d) + " <= " + g(kPreTol));
}

// ─── P1–P5 ───────────────────────────────────────────────────────────────────

LaneDetectorConfig cropCfg(float top, float bottom) {
    LaneDetectorConfig c;
    c.inputCropTop = top;
    c.inputCropBottom = bottom;
    return c;
}

void testP1Reference(const TestEnv&) {
    struct Geo { const char* name; int w, h; PreParams p; };
    const LaneGeometry g1 = computeLaneGeometry(1456, 1088, cropCfg(0.5f, 0.2059f));  // IMX296 band
    const LaneGeometry g5 = computeLaneGeometry(1920, 1080, cropCfg(0.5f, 0.2059f));  // UGREEN band
    const LaneGeometry gf = computeLaneGeometry(1920, 1080, cropCfg(0.0f, 0.0f));     // full frame
    const std::vector<Geo> geos = {
        { "identity 1600x320",      1600, 320, PreParams{} },
        { "IMX296 band 1456x320",   static_cast<int>(g1.branchW), static_cast<int>(g1.branchH), paramsFor(g1) },
        { "UGREEN band 1920x318",   static_cast<int>(g5.branchW), static_cast<int>(g5.branchH), paramsFor(g5) },
        { "full frame 1920x1080",   static_cast<int>(gf.branchW), static_cast<int>(gf.branchH), paramsFor(gf) },
    };
    double worst = 0;
    for (const Geo& geo : geos) {
        comparePre(std::string("P1 random    ") + geo.name, randomFrame(geo.w, geo.h, 1234u + geo.w), geo.p, &worst);
        comparePre(std::string("P1 checker   ") + geo.name, checkerFrame(geo.w, geo.h, 7), geo.p, &worst);
        comparePre(std::string("P1 gradient  ") + geo.name, gradientFrame(geo.w, geo.h), geo.p, &worst);
    }
    note("P1 worst max |gpu - ref| over all cases: " + g(worst) + " (tolerance " + g(kPreTol) + ")");

    // Identity geometry samples pixel centres exactly: the output is the
    // normalised input, element for element.
    const Frame fr = randomFrame(1600, 320, 99u);
    const std::vector<float> gpu = gpuPre(fr, PreParams{});
    double d = gpu.empty() ? INFINITY : 0;
    Norm n;
    for (int y = 0; y < 320 && !gpu.empty(); ++y)
        for (int x = 0; x < 1600; ++x)
            for (int c = 0; c < 3; ++c)
                d = std::max(d, std::fabs(gpu[static_cast<size_t>(c) * 320 * 1600 + y * 1600 + x] -
                                          normValue(fr.at(x, y)[2 - c], c, n)));
    check(d <= 1e-5, "P1 identity geometry = normalised pixel, element for element (max |d| " + g(d) + ")");
}

void testP2Channels(const TestEnv&) {
    const int W = 1456, H = 320;
    PreParams p;   // horizontal resample 1456 -> 1600, no vertical
    const Norm& n = p.n;
    const size_t plane = static_cast<size_t>(p.dstW) * p.dstH;
    const char* names[3] = { "R", "G", "B" };
    for (int hot = 0; hot < 3; ++hot) {          // hot plane: 0 R, 1 G, 2 B
        Frame fr(W, H);
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) fr.at(x, y)[2 - hot] = 255;   // BGR byte of that colour
        const std::vector<float> out = gpuPre(fr, p);
        bool ok = !out.empty();
        double worst = 0;
        for (int c = 0; c < 3 && ok; ++c) {
            const double want = normValue(c == hot ? 255.0 : 0.0, c, n);
            for (size_t i = 0; i < plane; ++i)
                worst = std::max(worst, std::fabs(out[c * plane + i] - want));
        }
        check(ok && worst <= 1e-5, std::string("P2 pure ") + names[hot] + " frame -> only CHW plane " +
              std::to_string(hot) + " is 1.0 before normalisation (max |d| " + g(worst) + ")");
    }

    // Layout: red on the left half only, at identity geometry.  CHW puts it
    // in the first plane's left half; HWC or a swapped plane would not.
    Frame fr(1600, 320);
    for (int y = 0; y < 320; ++y)
        for (int x = 0; x < 800; ++x) fr.at(x, y)[2] = 255;
    const std::vector<float> out = gpuPre(fr, PreParams{});
    bool ok = !out.empty();
    for (int y = 0; y < 320 && ok; ++y)
        for (int x = 0; x < 1600 && ok; ++x) {
            const float r = out[static_cast<size_t>(y) * 1600 + x];
            const float gg = out[plane + static_cast<size_t>(y) * 1600 + x];
            const float b = out[2 * plane + static_cast<size_t>(y) * 1600 + x];
            ok = std::fabs(r - normValue(x < 800 ? 255 : 0, 0, n)) < 1e-5 &&
                 std::fabs(gg - normValue(0, 1, n)) < 1e-5 && std::fabs(b - normValue(0, 2, n)) < 1e-5;
        }
    check(ok, "P2 CHW layout: red left half lands in plane 0, columns 0..799, every row");

    // Per-channel normalisation constants are each applied to their own plane.
    PreParams q;
    q.n = Norm{ 0.1f, 0.2f, 0.3f, 0.5f, 0.25f, 0.125f };
    Frame grey(1600, 320, 128);
    const std::vector<float> o2 = gpuPre(grey, q);
    bool ok2 = !o2.empty();
    for (int c = 0; c < 3 && ok2; ++c)
        ok2 = std::fabs(o2[c * plane + 12345] - normValue(128, c, q.n)) < 1e-5;
    check(ok2, "P2 mean/std are per plane (custom 0.1/0.2/0.3, 0.5/0.25/0.125)");
}

void testP3Borders(const TestEnv&) {
    // Out-of-bounds reads: a black frame between white guard bands must give
    // an all-black output, for every geometry the library can produce
    // (upsample, downsample, ROI offsets near both ends).
    const LaneGeometry g2 = computeLaneGeometry(1280, 720, cropCfg(0.0f, 0.0f));
    const LaneGeometry g4 = computeLaneGeometry(1280, 720, cropCfg(0.5f, 0.2059f));
    const LaneGeometry gt = computeLaneGeometry(1456, 1088, cropCfg(0.5f, 0.0f));
    struct Case { const char* name; int w, h; PreParams p; };
    PreParams up2;  up2.yScale = 0.5f;               // 800x160 -> 1600x320
    PreParams down2; down2.yScale = 2.0f;            // 3200x640 -> 1600x320
    const std::vector<Case> cases = {
        { "2x upsample 800x160",       800,  160, up2 },
        { "2x downsample 3200x640",    3200, 640, down2 },
        { "G2 ROI 1280x720",           1280, 720, paramsFor(g2) },
        { "G4 band 1280x212",          1280, static_cast<int>(g4.branchH), paramsFor(g4) },
        { "IMX296 top-only 1456x544",  1456, static_cast<int>(gt.branchH), paramsFor(gt) },
    };
    const Norm n;
    for (const Case& c : cases) {
        const Frame black(c.w, c.h, 0);
        const std::vector<float> out = gpuPre(black, c.p, 1 << 16, 255);
        double worst = out.empty() ? INFINITY : 0;
        const size_t plane = static_cast<size_t>(c.p.dstW) * c.p.dstH;
        for (int pl = 0; pl < 3 && !out.empty(); ++pl)
            for (size_t i = 0; i < plane; ++i)
                worst = std::max(worst, std::fabs(out[pl * plane + i] - normValue(0, pl, n)));
        check(worst <= 1e-6, std::string("P3 no read outside the frame: ") + c.name +
              " between 255 guards stays black (max |d| " + g(worst) + ")");
    }

    // Edge replication: with a 2x upsample the outermost output pixels map
    // to source coordinate -0.25 / W-0.75 and must equal the source edge
    // pixel exactly (floor + clamp, no smear into the neighbour).
    const Frame fr = randomFrame(800, 160, 7u);
    const std::vector<float> out = gpuPre(fr, up2);
    const size_t plane = 1600u * 320u;
    double worst = out.empty() ? INFINITY : 0;
    auto px = [&](int pl, int x, int y) { return out[pl * plane + static_cast<size_t>(y) * 1600 + x]; };
    for (int pl = 0; pl < 3 && !out.empty(); ++pl) {
        worst = std::max(worst, std::fabs(px(pl, 0, 0)       - normValue(fr.at(0, 0)[2 - pl], pl, n)));
        worst = std::max(worst, std::fabs(px(pl, 1599, 0)    - normValue(fr.at(799, 0)[2 - pl], pl, n)));
        worst = std::max(worst, std::fabs(px(pl, 0, 319)     - normValue(fr.at(0, 159)[2 - pl], pl, n)));
        worst = std::max(worst, std::fabs(px(pl, 1599, 319)  - normValue(fr.at(799, 159)[2 - pl], pl, n)));
    }
    check(worst <= 1e-5, "P3 2x upsample: the four corner outputs equal the source corner pixels (max |d| " + g(worst) + ")");
    double edge = 0;
    for (int y = 0; y < 320 && !out.empty(); y += 2)   // even rows: sy = y/2 - 0.25 -> 75/25 blend of rows
        for (int pl = 0; pl < 3; ++pl) {
            const int sy0 = std::max(0, y / 2 - 1), sy1 = y / 2;
            const double fy = y == 0 ? 0.0 : 0.75;
            const double v = (1 - fy) * fr.at(0, sy0)[2 - pl] + fy * fr.at(0, sy1)[2 - pl];
            edge = std::max(edge, std::fabs(px(pl, 0, y) - normValue(v, pl, n)));
        }
    check(edge <= 1e-4, "P3 left output column is the clamped source column 0, rows interpolated (max |d| " + g(edge) + ")");
    comparePre("P3 2x upsample random", fr, up2);
    comparePre("P3 2x downsample random", randomFrame(3200, 640, 8u), down2);
}

void testP4TrainingRoi(const TestEnv&) {
    // G2: no branch crop, 1280x720.  The preprocess reproduces Resize(H/0.6)
    // then keep the bottom 320 rows, as a source-side ROI.
    const LaneGeometry geo = computeLaneGeometry(1280, 720, cropCfg(0.0f, 0.0f));
    const double scale = 720.0 / 533.0;
    check(geo.resizedH == 533 && geo.cropTopRows == 213 && std::fabs(geo.preYScale - scale) < 1e-6 &&
              std::fabs(geo.preYOff - 213.0 * scale) < 1e-3,
          "P4 G2 ROI: preYScale 720/533 = " + f(geo.preYScale, 5) + ", preYOff 213*scale = " + f(geo.preYOff, 3));
    const PreParams p = paramsFor(geo);
    comparePre("P4 G2 random", randomFrame(1280, 720, 4u), p);
    comparePre("P4 G2 gradient", gradientFrame(1280, 720), p);

    // Closed form on step frames (green channel):
    //   rows < 287 white, rest black -> model row 0 (source row ~287.9) is black,
    //   so the top 40% of the frame never reaches the model;
    //   only row 719 white -> model row 319 is its fy-weighted blend.
    const Norm n;
    const double sy0 = geo.preYOff + 0.5 * geo.preYScale - 0.5;
    const double syL = geo.preYOff + 319.5 * geo.preYScale - 0.5;
    note("P4 model rows 0 / 319 sample source rows " + f(sy0, 3) + " / " + f(syL, 3) +
         " (the ROI is the bottom " + f((720.0 - geo.preYOff) / 720.0 * 100.0, 1) + "% of the frame)");
    Frame top(1280, 720, 0);
    for (int y = 0; y < 287; ++y)
        for (int x = 0; x < 1280; ++x) top.at(x, y)[1] = 255;
    const std::vector<float> o1 = gpuPre(top, p);
    const size_t plane = 1600u * 320u;
    double worst = o1.empty() ? INFINITY : 0;
    for (int x = 0; x < 1600 && !o1.empty(); ++x)
        worst = std::max(worst, std::fabs(o1[plane + x] - normValue(0, 1, n)));
    check(worst <= 1e-5, "P4 model row 0 sees no source row above 287 (top of the training crop)");

    Frame bottom(1280, 720, 0);
    for (int x = 0; x < 1280; ++x) bottom.at(x, 719)[1] = 255;
    const std::vector<float> o2 = gpuPre(bottom, p);
    const double fy = syL - std::floor(syL);
    const double want = normValue(255.0 * fy, 1, n);
    const double got = o2.empty() ? NAN : o2[plane + 319u * 1600u + 640];
    check(std::fabs(got - want) <= 1e-4, "P4 model row 319 = " + f(fy, 4) + " x source row 719 (got " +
          f(got, 5) + ", want " + f(want, 5) + ")");
}

void testP5VerticalUpsample(const TestEnv&) {
    // G4: 1280x720 at the default crops -> a 212-row band stretched to 320
    // model rows (preYScale 0.6625, no offset).
    const LaneGeometry geo = computeLaneGeometry(1280, 720, cropCfg(0.5f, 0.2059f));
    check(geo.branchH == 212 && std::fabs(geo.preYScale - 0.6625f) < 1e-6 && geo.preYOff == 0.0f,
          "P5 G4 band 1280x212, preYScale 0.6625, preYOff 0");
    const PreParams p = paramsFor(geo);
    comparePre("P5 G4 random", randomFrame(1280, 212, 5u), p);
    comparePre("P5 G4 checker", checkerFrame(1280, 212, 5), p);

    // Linear ramp G = source row (0..211): every model row m must read
    // clamp((m + 0.5)·0.6625 − 0.5, 0, 211) exactly (bilinear is exact on a
    // linear ramp; the clamp holds the first row at 0 and the last at 211).
    Frame ramp(1280, 212, 0);
    for (int y = 0; y < 212; ++y)
        for (int x = 0; x < 1280; ++x) ramp.at(x, y)[1] = static_cast<uint8_t>(y);
    const std::vector<float> out = gpuPre(ramp, p);
    const Norm n;
    double worst = out.empty() ? INFINITY : 0;
    for (int m = 0; m < 320 && !out.empty(); ++m) {
        const double sy = std::max(0.0, std::min(211.0, (m + 0.5) * static_cast<double>(geo.preYScale) - 0.5));
        for (int x : { 0, 777, 1599 })
            worst = std::max(worst, std::fabs(out[1600u * 320u + static_cast<size_t>(m) * 1600 + x] - normValue(sy, 1, n)));
    }
    check(worst <= 1e-4, "P5 vertical ramp: model row m reads source row clamp((m+0.5)*0.6625-0.5) (max |d| " + g(worst) + ")");
}

// ─── engines ─────────────────────────────────────────────────────────────────

class QuietTrtLogger : public nvinfer1::ILogger {
public:
    std::vector<std::string> errors;
    void log(Severity s, const char* msg) noexcept override {
        if (s <= Severity::kERROR) errors.emplace_back(msg);
    }
};

std::vector<char> readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) return {};
    const std::streamsize n = in.tellg();
    std::vector<char> buf(static_cast<size_t>(n));
    in.seekg(0);
    in.read(buf.data(), n);
    return buf;
}

struct TrtEngine {
    QuietTrtLogger logger;
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    double loadMs = 0;
    explicit TrtEngine(const std::string& path) {
        const double t0 = nowMs();
        const std::vector<char> blob = readFile(path);
        runtime.reset(nvinfer1::createInferRuntime(logger));
        if (!blob.empty() && runtime)
            engine.reset(runtime->deserializeCudaEngine(blob.data(), blob.size()));
        loadMs = nowMs() - t0;
    }
};

std::string dims(const nvinfer1::Dims& d) {
    std::string s = "(";
    for (int i = 0; i < d.nbDims; ++i) s += (i ? "," : "") + std::to_string(d.d[i]);
    return s + ")";
}

// IO contract of a UFLD v2 engine; @p wantIn / @p wantOut are the expected
// data types (kFLOAT for a production engine).
void checkIoContract(const std::string& tag, nvinfer1::ICudaEngine& e,
                     nvinfer1::DataType wantIn, nvinfer1::DataType wantOut) {
    struct Want { const char* name; nvinfer1::TensorIOMode mode; std::vector<int64_t> shape; };
    const std::vector<Want> want = {
        { "input",     nvinfer1::TensorIOMode::kINPUT,  { 1, 3, 320, 1600 } },
        { "loc_row",   nvinfer1::TensorIOMode::kOUTPUT, { 1, 200, 72, 4 } },
        { "loc_col",   nvinfer1::TensorIOMode::kOUTPUT, { 1, 100, 81, 4 } },
        { "exist_row", nvinfer1::TensorIOMode::kOUTPUT, { 1, 2, 72, 4 } },
        { "exist_col", nvinfer1::TensorIOMode::kOUTPUT, { 1, 2, 81, 4 } },
    };
    check(e.getNbIOTensors() == 5, tag + " has 5 IO tensors (got " + std::to_string(e.getNbIOTensors()) + ")");
    int inputs = 0;
    for (int i = 0; i < e.getNbIOTensors(); ++i)
        inputs += e.getTensorIOMode(e.getIOTensorName(i)) == nvinfer1::TensorIOMode::kINPUT;
    check(inputs == 1, tag + " has exactly one input");
    for (const Want& w : want) {
        bool present = false;
        for (int i = 0; i < e.getNbIOTensors(); ++i)
            present = present || std::strcmp(e.getIOTensorName(i), w.name) == 0;
        if (!present) {
            check(false, tag + " has tensor '" + w.name + "'");
            continue;
        }
        const nvinfer1::Dims d = e.getTensorShape(w.name);
        bool shapeOk = d.nbDims == static_cast<int>(w.shape.size());
        for (int k = 0; shapeOk && k < d.nbDims; ++k) shapeOk = d.d[k] == w.shape[static_cast<size_t>(k)];
        const bool isIn = w.mode == nvinfer1::TensorIOMode::kINPUT;
        const nvinfer1::DataType dt = e.getTensorDataType(w.name);
        const nvinfer1::TensorFormat fmt = e.getTensorFormat(w.name);
        check(e.getTensorIOMode(w.name) == w.mode && shapeOk &&
                  dt == (isIn ? wantIn : wantOut) && fmt == nvinfer1::TensorFormat::kLINEAR,
              tag + " " + w.name + ": " + (isIn ? "input " : "output ") + dims(d) + " dtype " +
                  std::to_string(static_cast<int>(dt)) + " format " + std::to_string(static_cast<int>(fmt)) +
                  " (want " + std::to_string(static_cast<int>(isIn ? wantIn : wantOut)) + ", kLINEAR 0)");
    }
}

LaneDetectorConfig engineCfg(const std::string& engine, LogSink& sink, float top = 0.5f, float bottom = 0.2059f) {
    LaneDetectorConfig c;
    c.enginePath = engine;
    c.inputCropTop = top;
    c.inputCropBottom = bottom;
    c.log = sink.callback();
    return c;
}

// Builds a detector, reporting (not throwing) a construction failure.
std::unique_ptr<LaneDetector> makeDetector(uint32_t w, uint32_t h, const LaneDetectorConfig& cfg,
                                           const std::string& what, std::string* err = nullptr) {
    try {
        return std::make_unique<LaneDetector>(w, h, cfg);
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        else check(false, what + ": construction threw: " + e.what());
    }
    return nullptr;
}

std::vector<uint8_t> branchFrame(const LaneDetector& det, uint32_t seed, bool road = true) {
    const LaneGeometry& geo = det.geometry();
    const Frame fr = road ? roadFrame(static_cast<int>(geo.branchW), static_cast<int>(geo.branchH), seed)
                          : randomFrame(static_cast<int>(geo.branchW), static_cast<int>(geo.branchH), seed);
    return fr.px;
}

void testE1IoContract(const TestEnv& env) {
    TrtEngine real(env.engine);
    if (!real.engine) {
        check(false, "E1 real engine deserializes: " +
              (real.logger.errors.empty() ? std::string("?") : real.logger.errors.front()));
        return;
    }
    checkIoContract("E1 real engine", *real.engine, nvinfer1::DataType::kFLOAT, nvinfer1::DataType::kFLOAT);
    note("E1 real engine: TRT API load " + f(real.loadMs, 0) + " ms, getDeviceMemorySizeV2 " +
         f(real.engine->getDeviceMemorySizeV2() / 1048576.0, 1) + " MiB, " +
         std::to_string(real.engine->getNbLayers()) + " layers");
    if (env.tinyEngine.empty() || !fileExists(env.tinyEngine)) {
        note("E1 tiny engine: skipped (no --tiny)");
        return;
    }
    TrtEngine tiny(env.tinyEngine);
    check(tiny.engine != nullptr, "E1 tiny engine deserializes");
    if (tiny.engine)
        checkIoContract("E1 tiny engine", *tiny.engine, nvinfer1::DataType::kFLOAT, nvinfer1::DataType::kFLOAT);
}

void testE2Fp16Io(const TestEnv& env) {
    bool any = false;
    if (!env.fp16InEngine.empty() && fileExists(env.fp16InEngine)) {
        any = true;
        {   // Fixture: the engine really binds its input as fp16.
            TrtEngine e(env.fp16InEngine);
            check(e.engine && e.engine->getTensorDataType("input") == nvinfer1::DataType::kHALF,
                  "E2 fixture: --fp16-in engine's input is kHALF");
        }
        LogSink sink;
        std::string err;
        auto det = makeDetector(1600, 320, engineCfg(env.fp16InEngine, sink, 0.0f, 0.0f), "E2", &err);
        bugCheck("B10", det == nullptr,
                 "E2 an engine with an fp16 input binding is rejected at load" +
                 (det ? std::string(" (it loaded: the 1600x320x3 float buffer would be read as half)")
                      : ": " + err));
    } else {
        note("E2 fp16-input engine: skipped (no --fp16-in)");
    }
    if (!env.fp16OutEngine.empty() && fileExists(env.fp16OutEngine)) {
        any = true;
        {
            TrtEngine e(env.fp16OutEngine);
            check(e.engine && e.engine->getTensorDataType("loc_row") == nvinfer1::DataType::kHALF,
                  "E2 fixture: --fp16-out engine's outputs are kHALF");
        }
        LogSink sink;
        std::string err;
        auto det = makeDetector(1600, 320, engineCfg(env.fp16OutEngine, sink, 0.0f, 0.0f), "E2", &err);
        check(det == nullptr && err.find("not float32") != std::string::npos,
              "E2 control: fp16 outputs rejected at load (" + (det ? std::string("it loaded") : err) + ")");
    } else {
        note("E2 fp16-output control: skipped (no --fp16-out)");
    }
    if (!any) note("E2 skipped entirely: build the engines with tools/lane_eval/make_tiny_engine.py");
}

void testE3LoadCost(const TestEnv& env) {
    size_t free0 = 0, total = 0, free1 = 0, free2 = 0;
    cudaMemGetInfo(&free0, &total);
    const long avail0 = memAvailableKb();
    LogSink sink;
    const double t0 = nowMs();
    auto det = makeDetector(1456, 1088, engineCfg(env.engine, sink), "E3");
    const double loadMs = nowMs() - t0;
    if (!det) return;
    const long avail1 = memAvailableKb();
    cudaMemGetInfo(&free1, &total);
    check(loadMs < 30000, "E3 real engine detector constructed in " + f(loadMs, 0) + " ms (< 30 s)");
    note("E3 LaneDetector construction " + f(loadMs, 0) + " ms; MemAvailable -" +
         f((avail0 - avail1) / 1024.0, 1) + " MiB; cudaMemGetInfo free -" +
         f((static_cast<double>(free0) - free1) / 1048576.0, 1) + " MiB (unified RAM: the same pool)");
    std::error_code ec;
    note("E3 engine file " + f(static_cast<double>(std::filesystem::file_size(env.engine, ec)) / 1048576.0, 1) +
         " MiB (getDeviceMemorySizeV2, the per-context scratch, is in E1)");
    det.reset();
    const long avail2 = memAvailableKb();
    cudaMemGetInfo(&free2, &total);
    note("E3 after destruction: MemAvailable back to " + f((avail0 - avail2) / 1024.0, 1) +
         " MiB below the start; cudaMemGetInfo " + f((static_cast<double>(free0) - free2) / 1048576.0, 1) + " MiB below");
    check(free2 + (64u << 20) >= free0, "E3 destruction returns the device memory (within 64 MiB)");
}

void testE4BitIdentical(const TestEnv& env) {
    LogSink sink;
    auto det = makeDetector(1456, 1088, engineCfg(env.engine, sink), "E4");
    if (!det) return;
    const std::vector<uint8_t> a = branchFrame(*det, 11u);
    const std::vector<uint8_t> b = branchFrame(*det, 12u, false);
    LaneResult ra1, rb, ra2;
    LaneDebugTensors da1, db, da2;
    const bool ok = det->processFrame(a.data(), a.size(), 1, ra1, &da1) &&
                    det->processFrame(b.data(), b.size(), 2, rb, &db) &&
                    det->processFrame(a.data(), a.size(), 3, ra2, &da2);
    check(ok, "E4 three processFrame calls (A, B, A) succeed");
    if (!ok) return;
    check(sameBits(da1.input, da2.input), "E4 A twice: input tensor bit-identical");
    bool outs = true, differs = false;
    for (size_t i = 0; i < 4; ++i) {
        outs = outs && !da1.outputs[i].empty() && sameBits(da1.outputs[i], da2.outputs[i]);
        differs = differs || !sameBits(da1.outputs[i], db.outputs[i]);
    }
    check(outs, "E4 A twice (B in between): all four output tensors bit-identical");
    check(sameResult(ra1, ra2), "E4 A twice: LaneResult identical (numLanes " + std::to_string(ra1.numLanes) +
          ", ego " + std::to_string(ra1.currentLaneIndex) + ")");
    check(differs, "E4 sanity: a different frame B gives different outputs");
    check(ra1.timing.trtUs == -1.0f && ra1.timing.h2dUs == -1.0f && ra1.timing.decUs == -1.0f,
          "E4 collectTimings off: stage times stay -1");
    check(ra1.timing.pullMonoNs > 0 && ra1.timing.doneMonoNs >= ra1.timing.pullMonoNs && ra1.timing.ptsNs == 1,
          "E4 collectTimings off: pts and mono timestamps still set");
    int detected = 0;
    for (const auto& s : ra1.slots) detected += s.detected;
    note("E4 synthetic road frame: " + std::to_string(detected) + " slots detected, numLanes " +
         std::to_string(ra1.numLanes) + ", nonFinite " + std::to_string(ra1.nonFinite));
}

void testE5Identity(const TestEnv& env) {
    const std::vector<char> blob = readFile(env.engine);
    check(!blob.empty(), "E5 engine file readable");
    note("E5 engine " + env.engine + ": " + std::to_string(blob.size()) + " bytes");
    note("E5 TensorRT: header " + std::to_string(NV_TENSORRT_MAJOR) + "." + std::to_string(NV_TENSORRT_MINOR) +
         "." + std::to_string(NV_TENSORRT_PATCH) + ", library " + std::to_string(getInferLibVersion()));
    if (blob.size() >= 32) {
        std::string hex;
        char b[4];
        for (int i = 0; i < 32; ++i) {
            std::snprintf(b, sizeof(b), "%02x", static_cast<unsigned char>(blob[static_cast<size_t>(i)]));
            hex += b;
        }
        note("E5 plan header " + hex);
    }
    // sha256 via coreutils (optional: absent tool -> no line).
    const std::string cmd = "sha256sum '" + env.engine + "' 2>/dev/null";
    if (FILE* p = ::popen(cmd.c_str(), "r")) {
        char line[160] = {};
        if (std::fgets(line, sizeof(line), p)) {
            std::string s(line);
            note("E5 sha256 " + s.substr(0, s.find(' ')));
        }
        ::pclose(p);
    }
}

// ─── processFrame / sink ─────────────────────────────────────────────────────

void testPF1Determinism(const TestEnv& env) {
    LogSink sink;
    LaneDetectorConfig cfg = engineCfg(env.engine, sink);
    cfg.collectTimings = true;
    auto det = makeDetector(1456, 1088, cfg, "PF1");
    if (!det) return;
    const std::vector<uint8_t> fr = branchFrame(*det, 21u);
    LaneResult first;
    LaneDebugTensors firstDbg;
    bool allOk = true, same = true, timed = true, seqOk = true;
    std::vector<float> trt, pre, dec;
    const uint64_t seq0 = det->processedFrameCount();
    for (int i = 0; i < 50; ++i) {
        LaneResult r;
        LaneDebugTensors dbg;
        const bool ok = det->processFrame(fr.data(), fr.size(), 1000 + i, r, i % 10 == 0 ? &dbg : nullptr);
        allOk = allOk && ok;
        if (!ok) break;
        if (i == 0) {
            first = r;
            firstDbg = dbg;
        } else {
            same = same && sameResult(first, r);
            if (i % 10 == 0)
                for (size_t k = 0; k < 4; ++k) same = same && sameBits(firstDbg.outputs[k], dbg.outputs[k]);
        }
        const LaneTiming& t = r.timing;
        timed = timed && t.h2dUs >= 0 && t.preUs >= 0 && t.trtUs > 0 && t.d2hUs >= 0 && t.decUs >= 0 &&
                t.ptsNs == 1000 + i && t.pullMonoNs > 0 && t.doneMonoNs >= t.pullMonoNs;
        seqOk = seqOk && r.sequence == seq0 + static_cast<uint64_t>(i) + 1;
        trt.push_back(t.trtUs);
        pre.push_back(t.preUs);
        dec.push_back(t.decUs);
    }
    check(allOk, "PF1 50 processFrame runs on one frame all valid");
    check(same, "PF1 all 50 results identical (outputs bit-identical on every 10th)");
    check(timed, "PF1 collectTimings: h2d/pre/trt/d2h/dec >= 0, pts carried, pull <= done");
    check(seqOk && det->processedFrameCount() == seq0 + 50 && det->stats().processed == seq0 + 50,
          "PF1 sequence advances by one per frame; processedFrameCount / stats agree");
    const LaneResult polled = det->poll();
    check(polled.sequence == seq0 + 50 && det->hasFreshResult(2000), "PF1 poll() holds the last result; fresh");
    if (!trt.empty()) {
        auto med = [](std::vector<float> v) { std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
        note("PF1 medians: preprocess " + f(med(pre), 0) + " us, TensorRT " + f(med(trt), 0) +
             " us, decode " + f(med(dec), 0) + " us");
    }
}

void testPF2RefusedWhileRunning(const TestEnv& env) {
    LogSink sink;
    auto det = makeDetector(1456, 1088, engineCfg(env.engine, sink), "PF2");
    if (!det) return;
    const std::vector<uint8_t> fr = branchFrame(*det, 31u);
    LaneResult r;
    check(det->processFrame(fr.data(), fr.size(), 0, r), "PF2 processFrame works before start()");
    const uint64_t n0 = det->stats().processed;   // lifetime (processedFrameCount() restarts at start())

    // No bin: the thread starts and finds no appsink.
    sink.clear();
    det->start();
    LaneResult r2;
    LaneDebugTensors dbg;
    const bool ran = det->processFrame(fr.data(), fr.size(), 0, r2, &dbg);
    check(!ran && !r2.valid && dbg.input.empty(), "PF2 processFrame refused after start() (no bin)");
    check(sink.has("E lane processFrame refused"), "PF2 refusal logged as ERROR");
    check(det->stats().processed == n0, "PF2 the refused frame is not counted");
    det->stop();

    // With a bin that is never put in a pipeline: the thread polls an
    // unstarted appsink.
    GstElement* bin = det->createBin();
    check(bin != nullptr, "PF2 createBin() before start");
    if (bin) gst_object_ref_sink(bin);
    sink.clear();
    det->start();
    const bool ran2 = det->processFrame(fr.data(), fr.size(), 0, r2);
    check(!ran2 && sink.has("E lane processFrame refused"), "PF2 processFrame refused while the thread polls an idle appsink");
    det->stop();
    if (bin) gst_object_unref(bin);

    check(det->processFrame(fr.data(), fr.size(), 0, r2) && det->stats().processed == n0 + 1,
          "PF2 processFrame works again after stop()");
}

void testPF3ResultSink(const TestEnv& env) {
    LogSink sink;
    auto det = makeDetector(1456, 1088, engineCfg(env.engine, sink), "PF3");
    if (!det) return;
    std::vector<LaneResult> got;
    check(det->setResultSink([&got](const LaneResult& r) { got.push_back(r); }), "PF3 setResultSink accepted before start()");
    const std::vector<uint8_t> a = branchFrame(*det, 41u);
    const std::vector<uint8_t> b = branchFrame(*det, 42u, false);
    std::vector<LaneResult> outs;
    bool ok = true;
    for (int i = 0; i < 6; ++i) {
        LaneResult r;
        const std::vector<uint8_t>& fr = i % 2 ? b : a;
        ok = ok && det->processFrame(fr.data(), fr.size(), 500 + i, r);
        outs.push_back(r);
    }
    bool match = ok && got.size() == outs.size();
    for (size_t i = 0; match && i < got.size(); ++i)
        match = sameResult(got[i], outs[i]) && got[i].sequence == outs[i].sequence &&
                got[i].timing.ptsNs == outs[i].timing.ptsNs;
    check(match, "PF3 the sink received every processFrame result, in order, identical (" +
          std::to_string(got.size()) + " of 6)");

    // A failed frame does not reach the sink.
    LaneDetectorTestHook::injectFault(*det, LaneDetectorTestHook::Fault::Enqueue, 1);
    LaneResult bad;
    const size_t before = got.size();
    check(!det->processFrame(a.data(), a.size(), 0, bad) && got.size() == before,
          "PF3 a failed frame (injected enqueue fault) is not delivered to the sink");

    sink.clear();
    det->start();
    int other = 0;
    check(!det->setResultSink([&other](const LaneResult&) { ++other; }), "PF3 setResultSink refused after start()");
    check(sink.has("E lane setResultSink refused"), "PF3 refusal logged as ERROR");
    det->stop();
    LaneResult r;
    det->processFrame(a.data(), a.size(), 0, r);
    check(other == 0 && got.size() == before + 1, "PF3 the refused sink was not installed; the original still receives");
    const bool afterStop = det->setResultSink(nullptr);
    note(std::string("PF3 setResultSink after stop(): ") + (afterStop ? "accepted" : "refused"));
}

// ─── tiny engine ─────────────────────────────────────────────────────────────

// lane_test_unit.cpp's fourLanes() fixture, which make_tiny_engine.py bakes
// into the tiny engine's constant outputs.
struct Fixture {
    UfldDims d;
    std::vector<float> locRow, locCol, existRow, existCol;
    Fixture()
        : locRow(static_cast<size_t>(d.numCellRow) * d.numRowAnchors * d.numLanes, 0.0f),
          locCol(static_cast<size_t>(d.numCellCol) * d.numColAnchors * d.numLanes, 0.0f),
          existRow(2u * d.numRowAnchors * d.numLanes, 0.0f),
          existCol(2u * d.numColAnchors * d.numLanes, 0.0f) {
        for (int k = 0; k < d.numRowAnchors; ++k)
            for (int l = 0; l < d.numLanes; ++l) er(0, k, l) = 1.0f;
        for (int k = 0; k < d.numColAnchors; ++k)
            for (int l = 0; l < d.numLanes; ++l) ec(0, k, l) = 1.0f;
        for (int k = 0; k < d.numRowAnchors; ++k) {
            rowPoint(1, k, 69);
            rowPoint(2, k, 129);
        }
        colLine(0, 0, 30, [](int k) { return 0.4 + 0.6 * (1.0 - k / 29.0); });
        colLine(3, 51, 81, [](int k) { return 0.4 + 0.6 * ((k - 51) / 29.0); });
    }
    float& lr(int g, int k, int l) { return locRow[(static_cast<size_t>(g) * d.numRowAnchors + k) * d.numLanes + l]; }
    float& lc(int g, int k, int l) { return locCol[(static_cast<size_t>(g) * d.numColAnchors + k) * d.numLanes + l]; }
    float& er(int c, int k, int l) { return existRow[(static_cast<size_t>(c) * d.numRowAnchors + k) * d.numLanes + l]; }
    float& ec(int c, int k, int l) { return existCol[(static_cast<size_t>(c) * d.numColAnchors + k) * d.numLanes + l]; }
    void rowPoint(int lane, int k, int g) {
        er(0, k, lane) = 0.0f; er(1, k, lane) = 1.0f;
        for (int c = 0; c < d.numCellRow; ++c) lr(c, k, lane) = -10.0f;
        lr(g, k, lane) = 10.0f;
    }
    void colPoint(int lane, int k, int g) {
        ec(0, k, lane) = 0.0f; ec(1, k, lane) = 1.0f;
        for (int c = 0; c < d.numCellCol; ++c) lc(c, k, lane) = -10.0f;
        lc(g, k, lane) = 10.0f;
    }
    template <class Y>
    void colLine(int lane, int k0, int k1, Y y) {
        for (int k = k0; k < k1; ++k) {
            const int g = static_cast<int>(std::lround(y(k) * (d.numCellCol - 1) - 0.5));
            colPoint(lane, k, std::max(0, std::min(d.numCellCol - 1, g)));
        }
    }
    // What LaneDetector::decode() makes of these tensors at width @p srcW.
    LaneResult expected(float srcW) const {
        UfldDecodeParams p;
        p.srcW = srcW;
        const UfldDecodeOut o = decodeUfldV2(locRow.data(), locCol.data(), existRow.data(), existCol.data(), d, p);
        const LaneEgo e = egoFromSlots(o.slots, srcW * 0.5f);
        LaneResult r;
        r.valid = true;
        r.slots = o.slots;
        r.nonFinite = o.nonFinite;
        r.numLanes = e.numLanes;
        r.currentLaneIndex = e.currentLaneIndex;
        r.lateralOffset = e.lateralOffset;
        r.lateralValid = e.lateralValid;
        return r;
    }
    bool sameAs(const LaneDebugTensors& t) const {
        return sameBits(t.outputs[0], locRow) && sameBits(t.outputs[1], locCol) &&
               sameBits(t.outputs[2], existRow) && sameBits(t.outputs[3], existCol);
    }
};

bool haveTiny(const TestEnv& env, const char* test) {
    if (!env.tinyEngine.empty() && fileExists(env.tinyEngine)) return true;
    note(std::string(test) + " skipped: no --tiny engine (tools/lane_eval/make_tiny_engine.py)");
    return false;
}

void testT1TinyDecode(const TestEnv& env) {
    if (!haveTiny(env, "T1")) return;
    LogSink sink;
    // 1600-px-wide source so the decode's x domain equals the fixture's.
    auto det = makeDetector(1600, 720, engineCfg(env.tinyEngine, sink, 0.5f, 0.0f), "T1");
    if (!det) return;
    const Fixture fx;
    const LaneResult want = fx.expected(1600.0f);
    check(want.numLanes == 3 && want.currentLaneIndex == 1 && want.lateralValid &&
              std::fabs(want.lateralOffset) < 0.02f,
          "T1 fixture decodes on the CPU to 3 lanes, ego 1, offset ~0 (" + f(want.lateralOffset, 4) + ")");
    for (uint32_t seed : { 1u, 2u }) {
        const std::vector<uint8_t> fr = branchFrame(*det, seed, seed == 2);
        LaneResult r;
        LaneDebugTensors dbg;
        const bool ok = det->processFrame(fr.data(), fr.size(), seed, r, &dbg);
        check(ok, "T1 processFrame on the tiny engine (frame " + std::to_string(seed) + ")");
        if (!ok) continue;
        check(fx.sameAs(dbg), "T1 TensorRT output buffers carry the fixture bit for bit");
        check(sameResult(r, want), "T1 LaneResult equals the CPU decode of the fixture bit for bit");
        check(r.numLanes == 3 && r.currentLaneIndex == 1 && r.lateralValid,
              "T1 4 boundaries -> 3 lanes, ego lane 1, lateral offset valid");
        check(std::fabs(r.slots[1].xRef - 558.79f) < 0.05f && std::fabs(r.slots[2].xRef - 1041.21f) < 0.05f,
              "T1 ego boundaries at 558.79 / 1041.21 px (got " + f(r.slots[1].xRef, 2) + " / " +
                  f(r.slots[2].xRef, 2) + ")");
    }
}

void testT2RunModelSized(const TestEnv& env) {
    if (!haveTiny(env, "T2")) return;
    LogSink sink;
    // A 1600x320 branch (no crop): processFrame resamples (training ROI,
    // effRatio 0.6); runModelSized must bypass that with an identity map.
    auto det = makeDetector(1600, 320, engineCfg(env.tinyEngine, sink, 0.0f, 0.0f), "T2");
    if (!det) return;
    const Frame fr = randomFrame(1600, 320, 77u);
    LaneResult rp, rm, rp2;
    LaneDebugTensors dp, dm, dp2;
    const bool okP = det->processFrame(fr.px.data(), fr.px.size(), 0, rp, &dp);
    const bool okM = LaneDetectorTestHook::runModelSized(*det, fr.px.data(), fr.px.size(), rm, &dm);
    const bool okP2 = det->processFrame(fr.px.data(), fr.px.size(), 0, rp2, &dp2);
    check(okP && okM && okP2, "T2 processFrame, runModelSized, processFrame on a 1600x320 branch");
    if (!(okP && okM && okP2)) return;
    const Fixture fx;
    check(sameResult(rm, fx.expected(1600.0f)) && fx.sameAs(dm), "T2 runModelSized result = fixture decode");
    // Identity preprocess: input tensor = normalised frame, element for element.
    const Norm n;
    const bool sized = dm.input.size() == 3u * 1600u * 320u;
    double worst = sized ? 0 : INFINITY;
    for (int y = 0; y < 320 && sized; ++y)
        for (int x = 0; x < 1600; ++x)
            for (int c = 0; c < 3; ++c)
                worst = std::max(worst, std::fabs(dm.input[static_cast<size_t>(c) * 512000 + y * 1600 + x] -
                                                  normValue(fr.at(x, y)[2 - c], c, n)));
    check(worst <= 1e-5, "T2 runModelSized input tensor = normalised frame (identity geometry, max |d| " + g(worst) + ")");
    check(!sameBits(dp.input, dm.input), "T2 processFrame's input differs (training ROI, preYScale " +
          f(det->geometry().preYScale, 4) + ")");
    check(sameBits(dp.input, dp2.input), "T2 runModelSized restores the ROI: processFrame input unchanged afterwards");
    check(rm.sequence == rp.sequence + 1 && rp2.sequence == rm.sequence + 1,
          "T2 runModelSized results are numbered like processFrame's");

    det->start();
    LaneResult r;
    check(!LaneDetectorTestHook::runModelSized(*det, fr.px.data(), fr.px.size(), r),
          "T2 runModelSized refused while the inference thread runs");
    det->stop();
    check(LaneDetectorTestHook::runModelSized(*det, fr.px.data(), fr.px.size(), r), "T2 accepted again after stop()");
    check(!LaneDetectorTestHook::runModelSized(*det, fr.px.data(), fr.px.size() - 3, r), "T2 wrong byte count refused");

    // A branch that is not model-sized is refused.
    auto other = makeDetector(1456, 1088, engineCfg(env.tinyEngine, sink), "T2 1456x320");
    if (!other) return;
    const Frame fr2 = randomFrame(1600, 320, 78u);
    check(!LaneDetectorTestHook::runModelSized(*other, fr2.px.data(), fr2.px.size(), r),
          "T2 runModelSized refused on a 1456x320 branch");
}


// ─── T2: appsrc-fed pipelines ────────────────────────────────────────────────
//
// A Feed is "appsrc ! [conversion] ! <detector bin>" in a pipeline the test
// owns: frames with controlled content, PTS and wall-clock pacing, no camera.

bool waitFor(const std::function<bool()>& pred, int timeoutMs) {
    const double t0 = nowMs();
    while (!pred()) {
        if (nowMs() - t0 > timeoutMs) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

struct Feed {
    GstElement*  pipe = nullptr;
    GstElement*  src  = nullptr;
    GstVideoInfo info{};
    GstClockTime frameDur = GST_SECOND / 30;
    std::thread  th;
    std::atomic<bool> feeding{false};
    std::atomic<long> pushed{0};

    // @p bin is sunk into the pipeline (pass createBin()'s floating ref).
    // @p quietLink: a refused link is an expected outcome (no check() line).
    bool build(const std::string& srcCaps, const std::string& conv, GstElement* bin,
               bool doTimestamp = false, bool quietLink = false) {
        if (!bin) return false;
        const std::string desc = std::string("appsrc name=src is-live=true format=time do-timestamp=") +
            (doTimestamp ? "true" : "false") + " caps=\"" + srcCaps + "\" ! " +
            (conv.empty() ? std::string() : conv + " ! ") + "identity name=tail";
        GError* err = nullptr;
        pipe = gst_parse_launch(desc.c_str(), &err);
        if (!pipe || err) {
            check(false, std::string("feed pipeline parse: ") + (err ? err->message : "?"));
            if (err) g_error_free(err);
            gst_object_unref(gst_object_ref_sink(bin));
            if (pipe) { gst_object_unref(pipe); pipe = nullptr; }
            return false;
        }
        src = gst_bin_get_by_name(GST_BIN(pipe), "src");
        GstElement* tail = gst_bin_get_by_name(GST_BIN(pipe), "tail");
        gst_bin_add(GST_BIN(pipe), bin);
        const bool linked = gst_element_link(tail, bin);
        gst_object_unref(tail);
        GstCaps* caps = gst_caps_from_string(srcCaps.c_str());
        const bool infoOk = gst_video_info_from_caps(&info, caps);
        gst_caps_unref(caps);
        if (infoOk && info.fps_n > 0) frameDur = gst_util_uint64_scale(GST_SECOND, info.fps_d, info.fps_n);
        if ((!linked && !quietLink) || !infoOk) check(false, "feed pipeline link (" + srcCaps + ")");
        return linked && infoOk;
    }
    bool play() {
        return pipe && gst_element_set_state(pipe, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE;
    }
    // One frame; @p fill writes into the mapped (stride-aware) video frame.
    bool push(int64_t ptsNs, const std::function<void(GstVideoFrame&, long)>& fill, long n) {
        GstBuffer* buf = gst_buffer_new_allocate(nullptr, info.size, nullptr);
        GstVideoFrame vf;
        if (!gst_video_frame_map(&vf, &info, buf, GST_MAP_WRITE)) {
            gst_buffer_unref(buf);
            return false;
        }
        fill(vf, n);
        gst_video_frame_unmap(&vf);
        if (ptsNs >= 0) {
            GST_BUFFER_PTS(buf) = static_cast<GstClockTime>(ptsNs);
            GST_BUFFER_DURATION(buf) = frameDur;
        }
        ++pushed;
        return gst_app_src_push_buffer(GST_APP_SRC(src), buf) == GST_FLOW_OK;   // takes buf
    }
    // Paced feeder: frame n leaves at t0 + n/fps + N(0, jitterMs) wall time,
    // with PTS n/fps exactly (a camera: exact timestamps, jittery delivery).
    void startFeeding(double jitterMs, std::function<void(GstVideoFrame&, long)> fill,
                      bool exactPts = true) {
        feeding = true;
        th = std::thread([this, jitterMs, fill, exactPts] {
            std::mt19937 rng(4242);
            std::normal_distribution<double> jit(0.0, jitterMs);
            const auto t0 = std::chrono::steady_clock::now();
            for (long n = 0; feeding; ++n) {
                const double ms = n * (frameDur / 1e6) + (jitterMs > 0 ? jit(rng) : 0.0);
                std::this_thread::sleep_until(t0 + std::chrono::microseconds(static_cast<long>(std::max(0.0, ms) * 1000)));
                if (!feeding) break;
                if (!push(exactPts ? static_cast<int64_t>(n * frameDur) : -1, fill, n)) break;
            }
        });
    }
    void stopFeeding() {
        feeding = false;
        if (th.joinable()) th.join();
    }
    bool busError(std::string* what = nullptr) {
        if (!pipe) return false;
        GstBus* bus = gst_element_get_bus(pipe);
        bool found = false;
        while (GstMessage* m = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR)) {
            if (!found && what) {
                GError* e = nullptr;
                gst_message_parse_error(m, &e, nullptr);
                *what = e ? e->message : "?";
                if (e) g_error_free(e);
            }
            found = true;
            gst_message_unref(m);
        }
        gst_object_unref(bus);
        return found;
    }
    void toNull() {
        stopFeeding();
        if (pipe) gst_element_set_state(pipe, GST_STATE_NULL);
    }
    ~Feed() {
        toNull();
        if (src) gst_object_unref(src);
        if (pipe) gst_object_unref(pipe);
    }
};

std::string bgrCaps(int w, int h, int fps) {
    return "video/x-raw,format=BGR,width=" + std::to_string(w) + ",height=" + std::to_string(h) +
           ",framerate=" + std::to_string(fps) + "/1";
}

// Mid-grey frames with a little per-frame variation (content is irrelevant
// to the lifecycle tests).
void fillGrey(GstVideoFrame& vf, long n) {
    auto* d = static_cast<uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&vf, 0));
    const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vf, 0);
    const int h = GST_VIDEO_FRAME_HEIGHT(&vf);
    std::memset(d, 100 + static_cast<int>(n % 40), static_cast<size_t>(stride) * h);
}

// A small system-memory detector (640x480, crops 0.5/0.2 -> 640x144 band).
LaneDetectorConfig smallCfg(const std::string& engine, LogSink& sink, uint32_t targetHz = 0,
                            uint32_t branchFps = 0) {
    LaneDetectorConfig c = engineCfg(engine, sink, 0.5f, 0.2f);
    c.sourceIsNVMM = false;
    c.targetHz = targetHz;
    c.branchMaxFps = branchFps;
    return c;
}

// The lane_infer thread's tid in this process, or −1.
int laneThreadTid(const char* name = "lane_infer") {
    DIR* d = ::opendir("/proc/self/task");
    if (!d) return -1;
    int tid = -1;
    while (dirent* e = ::readdir(d)) {
        if (e->d_name[0] == '.') continue;
        std::ifstream comm(std::string("/proc/self/task/") + e->d_name + "/comm");
        std::string c;
        std::getline(comm, c);
        if (c == name) { tid = std::atoi(e->d_name); break; }
    }
    ::closedir(d);
    return tid;
}

// utime + stime of a thread, in clock ticks.
long threadTicks(int tid) {
    std::ifstream in("/proc/self/task/" + std::to_string(tid) + "/stat");
    std::string line;
    std::getline(in, line);
    const size_t rp = line.rfind(')');
    if (rp == std::string::npos) return -1;
    std::istringstream rest(line.substr(rp + 2));
    std::string tok;
    long ut = 0, st = 0;
    for (int field = 3; rest >> tok; ++field) {   // field 3 = state
        if (field == 14) ut = std::atol(tok.c_str());
        if (field == 15) { st = std::atol(tok.c_str()); break; }
    }
    return ut + st;
}

// CPU share (% of one core) of the lane_infer thread over @p ms.
double laneThreadCpu(int ms) {
    const int tid = laneThreadTid();
    if (tid < 0) return -1;
    const long t0 = threadTicks(tid);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    const long t1 = threadTicks(tid);
    return 100.0 * (t1 - t0) / ::sysconf(_SC_CLK_TCK) / (ms / 1000.0);
}

long procStatusKb(const char* key) {
    std::ifstream in("/proc/self/status");
    std::string line;
    while (std::getline(in, line))
        if (line.rfind(key, 0) == 0) return std::atol(line.c_str() + std::strlen(key));
    return -1;
}

int countDir(const char* path) {
    DIR* d = ::opendir(path);
    if (!d) return -1;
    int n = 0;
    while (dirent* e = ::readdir(d)) n += e->d_name[0] != '.';
    ::closedir(d);
    return n;
}

// A running detector on a small BGR feed.
struct Rig {
    LogSink sink;
    std::unique_ptr<LaneDetector> det;
    Feed feed;
    bool ok = false;
    Rig(const std::string& engine, const char* what, uint32_t targetHz = 0, uint32_t branchFps = 0,
        int fps = 30) {
        det = makeDetector(640, 480, smallCfg(engine, sink, targetHz, branchFps), what);
        if (!det) return;
        ok = feed.build(bgrCaps(640, 480, fps), "", det->createBin()) && feed.play();
        if (!ok) { check(false, std::string(what) + ": feed pipeline plays"); return; }
        det->start();
        feed.startFeeding(0.0, fillGrey);
        ok = waitFor([&] { return det->processedFrameCount() >= 3; }, 5000);
        check(ok, std::string(what) + ": detector produces results from the appsrc feed");
    }
    ~Rig() {
        feed.toNull();       // pipeline first (cam.stop()), then the thread
        if (det) det->stop();
    }
};

// ─── L1–L14 ──────────────────────────────────────────────────────────────────

void testL1CreateBinWhileRunning(const TestEnv& env) {
    Rig rig(env.engine, "L1");
    if (!rig.ok) return;
    // The old bin stays owned by the running pipeline, so the swap the bug
    // makes cannot free the appsink the thread is pulling from (the test
    // shows the missing refusal without provoking the use-after-free).
    int accepted = 0;
    std::vector<GstElement*> extra;
    for (int i = 0; i < 20; ++i)
        if (GstElement* b = rig.det->createBin()) {
            ++accepted;
            extra.push_back(GST_ELEMENT(gst_object_ref_sink(b)));
        }
    bugCheck("B1", accepted == 0,
             "L1 createBin() is refused while the inference thread runs (accepted " +
             std::to_string(accepted) + " of 20; each swaps appsink_ under the thread)");
    check(rig.sink.count("E lane createBin refused") == 20, "L1 every refusal is logged as ERROR");
    const uint64_t n0 = rig.det->processedFrameCount();
    check(waitFor([&] { return rig.det->processedFrameCount() > n0 + 3; }, 3000),
          "L1 the running branch keeps producing results after the refusals");
    rig.feed.toNull();
    rig.det->stop();
    for (GstElement* b : extra) gst_object_unref(b);
}

void testL2SpinAfterStop(const TestEnv& env) {
    {
        Rig rig(env.engine, "L2 NULL");
        if (!rig.ok) return;
        rig.feed.toNull();                                    // pipeline gone, thread still running
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const double cpu = laneThreadCpu(2000);
        bugCheck("B2", cpu >= 0 && cpu < 5.0,
                 "L2 pipeline set to NULL, thread still running: lane_infer CPU " + f(cpu, 1) + "% of a core (< 5%)");
    }
    {
        Rig rig(env.engine, "L2 EOS");
        if (!rig.ok) return;
        rig.feed.stopFeeding();
        gst_app_src_end_of_stream(GST_APP_SRC(rig.feed.src));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        const double cpu = laneThreadCpu(2000);
        bugCheck("B2", cpu >= 0 && cpu < 5.0,
                 "L2 appsink at EOS, thread still running: lane_infer CPU " + f(cpu, 1) + "% of a core (< 5%)");
    }
}

void testL3RateGate(const TestEnv& env) {
    struct Case { uint32_t hz, fps; };
    using Gate = LaneDetectorConfig::RateGate;
    for (const Gate gate : { Gate::kPts, Gate::kLegacy })
    for (const Case c : { Case{ 20, 20 }, Case{ 20, 0 }, Case{ 0, 20 }, Case{ 20, 30 } }) {
        if (gate == Gate::kLegacy && c.hz == 0) continue;     // no gate in the path: same as kPts
        LogSink sink;
        LaneDetectorConfig cfg = smallCfg(env.engine, sink, c.hz, c.fps);
        cfg.rateGate = gate;
        auto det = makeDetector(640, 480, cfg, "L3");
        if (!det) return;
        Feed feed;
        if (!feed.build(bgrCaps(640, 480, 60), "", det->createBin()) || !feed.play()) return;
        det->start();
        feed.startFeeding(2.0, fillGrey);                     // 60 fps, sigma 2 ms wall jitter
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        const LaneStats s0 = det->stats();
        const double t0 = nowMs();
        std::this_thread::sleep_for(std::chrono::milliseconds(6000));
        const LaneStats s1 = det->stats();
        const double secs = (nowMs() - t0) / 1000.0;
        feed.toNull();
        det->stop();
        const double eff = (s1.processed - s0.processed) / secs;
        const double want = c.hz && c.fps ? std::min(c.hz, c.fps) : std::max(c.hz, c.fps);
        const std::string what = std::string("L3 ") + (gate == Gate::kPts ? "pts" : "legacy") +
            " gate, targetHz " + std::to_string(c.hz) + ", branchMaxFps " + std::to_string(c.fps) +
            " on 60 fps +-2 ms: " + f(eff, 1) + " Hz (want " + f(want, 0) + " +-10%; pulled " +
            std::to_string(s1.pulled - s0.pulled) + ", gated " + std::to_string(s1.gated - s0.gated) + ")";
        const bool ok = std::fabs(eff - want) <= 0.1 * want;
        if (gate == Gate::kPts) check(ok, what);
        else note(what + (ok ? "" : "  <- B3 aliasing (the kLegacy default)"));
    }
}

void testL4RestartState(const TestEnv& env) {
    Rig rig(env.engine, "L4");
    if (!rig.ok) return;
    rig.feed.toNull();
    rig.det->stop();
    const uint64_t before = rig.det->processedFrameCount();
    rig.det->start();                                         // no new frames will come
    const LaneResult p = rig.det->poll();
    const bool fresh = rig.det->hasFreshResult(60000);
    const uint64_t count = rig.det->processedFrameCount();
    rig.det->stop();
    bugCheck("B12", !p.valid && !fresh && count == 0,
             "L4 after stop()/start() with no new frames: poll() not valid, not fresh, count 0 (got valid " +
             std::to_string(p.valid) + ", fresh " + std::to_string(fresh) + ", count " + std::to_string(count) +
             " of " + std::to_string(before) + " from the previous run)");
}

void testL5NvmmSizeMismatch(const TestEnv& env) {
    LogSink sink;
    LaneDetectorConfig cfg = engineCfg(env.engine, sink);     // 1456x1088 NVMM, crops 0.5/0.2059
    cfg.sourceIsNVMM = true;
    cfg.targetHz = 0;
    cfg.branchMaxFps = 0;
    auto det = makeDetector(1456, 1088, cfg, "L5");
    if (!det) return;
    Feed feed;
    // The size is pinned after nvvidconv, as a camera tee's is: otherwise the
    // feed's own nvvidconv would simply scale to whatever the bin asks for.
    // Refusing the link outright (caps already incompatible) is the loudest
    // correct outcome.
    const bool linked = feed.build("video/x-raw,format=RGBA,width=1920,height=1080,framerate=30/1",
                                   "nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width=1920,height=1080",
                                   det->createBin(), false, /*quietLink=*/true);
    if (!linked) {
        bugCheck("B7", true, "L5 1920x1080 NVMM into a 1456x1088 bin: the link is refused (caps)");
        return;
    }
    const bool played = feed.play();
    det->start();
    feed.startFeeding(0.0, fillGrey);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    std::string err;
    const bool busErr = !played || feed.busError(&err);
    const LaneStats s = det->stats();
    feed.toNull();
    det->stop();
    bugCheck("B7", (busErr || s.sizeMismatch > 0) && s.processed == 0,
             "L5 1920x1080 NVMM into a 1456x1088 bin: caps error or size mismatch, never a result (bus error " +
             std::to_string(busErr) + (err.empty() ? "" : " '" + err + "'") + ", sizeMismatch " +
             std::to_string(s.sizeMismatch) + ", processed " + std::to_string(s.processed) + ")");
}

void testL6StridePadded(const TestEnv& env) {
    LogSink sink;
    LaneDetectorConfig cfg = engineCfg(env.engine, sink, 0.0f, 0.0f);
    cfg.sourceIsNVMM = false;
    cfg.targetHz = 0;
    cfg.branchMaxFps = 0;
    auto det = makeDetector(642, 480, cfg, "L6");             // 642*3 = 1926: rows padded to 1928
    if (!det) return;
    Feed feed;
    if (!feed.build(bgrCaps(642, 480, 30), "", det->createBin()) || !feed.play()) return;
    det->start();
    feed.startFeeding(0.0, fillGrey);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    const LaneStats s = det->stats();
    feed.toNull();
    det->stop();
    note("L6 642x480 BGR: GStreamer buffer " + std::to_string(feed.info.size) + " bytes (stride " +
         std::to_string(feed.info.stride[0]) + ") vs the detector's 642*480*3 = " + std::to_string(642 * 480 * 3));
    bugCheck("B15", s.processed > 0,
             "L6 a 642-wide frame (row stride padded) is processed (processed " + std::to_string(s.processed) +
             ", sizeMismatch " + std::to_string(s.sizeMismatch) + ")");
}

void testL7ErrorFlood(const TestEnv& env) {
    Rig rig(env.engine, "L7", 0, 0, 60);
    if (!rig.ok) return;
    const LaneStats s0 = rig.det->stats();
    rig.sink.clear();
    LaneDetectorTestHook::injectFault(*rig.det, LaneDetectorTestHook::Fault::Enqueue, 100);
    bool staleSeen = false;
    const bool reached = waitFor([&] {
        const LaneStats s = rig.det->stats();
        if (s.trtFail - s0.trtFail >= 90 && !staleSeen) staleSeen = !rig.det->hasFreshResult(500);
        return s.trtFail - s0.trtFail >= 100;
    }, 15000);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const LaneStats s1 = rig.det->stats();
    check(reached && s1.trtFail - s0.trtFail == 100, "L7 exactly 100 injected enqueue failures counted (" +
          std::to_string(s1.trtFail - s0.trtFail) + ")");
    check(staleSeen, "L7 during the burst hasFreshResult(500) is false");
    const size_t lines = rig.sink.count("lane enqueueV3 failed");
    bugCheck("B14", lines <= 4, "L7 100 failures in ~2 s log <= 3 ERROR lines + 1 summary (got " +
             std::to_string(lines) + ")");
    check(waitFor([&] { return rig.det->processedFrameCount() > s1.processed; }, 3000),
          "L7 inference resumes after the burst");
}

void testL8LogCallbackWhileRunning(const TestEnv& env) {
    Rig rig(env.engine, "L8", 0, 0, 30);
    if (!rig.ok) return;
    LogSink other;
    rig.sink.clear();
    check(!rig.det->setLogCallback(other.callback()), "L8 setLogCallback returns false while running");
    check(rig.sink.has("E lane setLogCallback refused"), "L8 the refusal is logged to the original sink");
    const LaneStats s0 = rig.det->stats();
    LaneDetectorTestHook::injectFault(*rig.det, LaneDetectorTestHook::Fault::Enqueue, 3);
    waitFor([&] { return rig.det->stats().trtFail - s0.trtFail >= 3; }, 5000);
    const size_t a = rig.sink.count("lane enqueueV3 failed");
    const size_t b = other.count("lane enqueueV3 failed");
    bugCheck("B13", a >= 1 && b == 0,
             "L8 setLogCallback while running is refused: the thread keeps logging to the original sink "
             "(original " + std::to_string(a) + " lines, new " + std::to_string(b) + ")");
}

void testL9RestartCycles(const TestEnv& env) {
    LogSink sink;
    auto det = makeDetector(640, 480, smallCfg(env.engine, sink), "L9");
    if (!det) return;
    long rss0 = 0, fds0 = 0, thr0 = 0;
    size_t free0 = 0, total = 0;
    int advanced = 0;
    const int kCycles = 50;
    for (int c = 0; c < kCycles; ++c) {
        if (c == 5) {                                  // baseline after warm-up
            rss0 = procStatusKb("VmRSS:");
            fds0 = countDir("/proc/self/fd");
            thr0 = countDir("/proc/self/task");
            cudaMemGetInfo(&free0, &total);
        }
        Feed feed;
        if (!feed.build(bgrCaps(640, 480, 30), "", det->createBin()) || !feed.play()) break;
        det->start();
        feed.startFeeding(0.0, fillGrey);
        // processedFrameCount() restarts at start() (B12 fix): > 0 = this run's frame.
        if (waitFor([&] { return det->processedFrameCount() > 0; }, 2000)) ++advanced;
        feed.toNull();                                 // cam.stop() first, then the thread
        det->stop();
    }
    const long rss1 = procStatusKb("VmRSS:");
    const long fds1 = countDir("/proc/self/fd");
    const long thr1 = countDir("/proc/self/task");
    size_t free1 = 0;
    cudaMemGetInfo(&free1, &total);
    check(advanced == kCycles, "L9 stop -> createBin -> relink -> start: results within 2 s in " +
          std::to_string(advanced) + " of " + std::to_string(kCycles) + " cycles");
    check(rss1 - rss0 < 20 * 1024, "L9 RSS growth over 45 cycles " + f((rss1 - rss0) / 1024.0, 1) + " MiB (< 20)");
    check(fds1 == fds0 && thr1 == thr0, "L9 fds " + std::to_string(fds0) + " -> " + std::to_string(fds1) +
          ", threads " + std::to_string(thr0) + " -> " + std::to_string(thr1) + " (stable)");
    const double dFree = (static_cast<double>(free0) - free1) / 1048576.0;
    check(dFree < 8.0, "L9 cudaMemGetInfo free drop " + f(dFree, 1) + " MiB (< 8)");
}

// Captures the first frames the inference thread maps (onFrame hook).
struct FrameGrab {
    std::mutex m;
    std::vector<std::vector<uint8_t>> frames;
    std::function<void(const uint8_t*, size_t)> cb() {
        return [this](const uint8_t* d, size_t n) {
            std::lock_guard<std::mutex> lk(m);
            if (frames.size() < 3) frames.emplace_back(d, d + n);
        };
    }
    std::vector<uint8_t> first() {
        std::lock_guard<std::mutex> lk(m);
        return frames.empty() ? std::vector<uint8_t>() : frames.front();
    }
};

// Feeds @p srcCaps through @p conv into a 1456x1088 detector (crops
// 0.5/0.2059) and returns the first frame the thread mapped.
std::vector<uint8_t> grabBand(const TestEnv& env, bool nvmm, const std::string& srcCaps,
                              const std::string& conv, std::function<void(GstVideoFrame&, long)> fill,
                              const std::string& tag) {
    LogSink sink;
    LaneDetectorConfig cfg = engineCfg(env.engine, sink);
    cfg.sourceIsNVMM = nvmm;
    cfg.targetHz = 0;
    cfg.branchMaxFps = 0;
    auto det = makeDetector(1456, 1088, cfg, tag);
    if (!det) return {};
    FrameGrab grab;
    LaneDetectorTestHook::onFrame(*det, grab.cb());
    Feed feed;
    if (!feed.build(srcCaps, conv, det->createBin()) || !feed.play()) return {};
    det->start();
    feed.startFeeding(0.0, std::move(fill));
    waitFor([&] { return !grab.first().empty(); }, 4000);
    std::string err;
    if (feed.busError(&err)) check(false, tag + ": pipeline error '" + err + "'");
    feed.toNull();
    det->stop();
    LaneDetectorTestHook::onFrame(*det, nullptr);
    return grab.first();
}

// Rows of the band that do not carry their source row number (R = row & 0xFF,
// G = row >> 8, B = 0), checked at three columns.
int badRows(const std::vector<uint8_t>& band, int w, int h, int topRow) {
    if (band.size() != static_cast<size_t>(w) * h * 3) return h;
    int bad = 0;
    for (int y = 0; y < h; ++y) {
        const int row = topRow + y;
        bool ok = true;
        for (int x : { 0, w / 2, w - 1 }) {
            const uint8_t* p = &band[(static_cast<size_t>(y) * w + x) * 3];
            ok = ok && p[0] == 0 && p[1] == (row >> 8) && p[2] == (row & 0xFF);
        }
        bad += !ok;
    }
    return bad;
}

void testL10GoldenRows(const TestEnv& env) {
    const LaneGeometry geo = computeLaneGeometry(1456, 1088, cropCfg(0.5f, 0.2059f));
    const int W = 1456, H = static_cast<int>(geo.branchH), top = static_cast<int>(geo.cropTopPx);

    // (a) Exact: RGBA in NVMM, so the VIC crop + RGBA->BGRx is a lossless copy.
    auto rgba = [](GstVideoFrame& vf, long) {
        auto* d = static_cast<uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&vf, 0));
        const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vf, 0);
        for (int y = 0; y < GST_VIDEO_FRAME_HEIGHT(&vf); ++y)
            for (int x = 0; x < GST_VIDEO_FRAME_WIDTH(&vf); ++x) {
                uint8_t* p = d + static_cast<size_t>(y) * stride + x * 4;
                p[0] = static_cast<uint8_t>(y & 0xFF); p[1] = static_cast<uint8_t>(y >> 8); p[2] = 0; p[3] = 255;
            }
    };
    const std::vector<uint8_t> a = grabBand(env, true, "video/x-raw,format=RGBA,width=1456,height=1088,framerate=30/1",
                                            "nvvidconv ! video/x-raw(memory:NVMM),format=RGBA", rgba, "L10 NVMM");
    const int badA = badRows(a, W, H, top);
    check(!a.empty() && badA == 0, "L10 NVMM path: band rows carry source rows " + std::to_string(top) + ".." +
          std::to_string(top + H - 1) + " exactly (" + std::to_string(badA) + " of " + std::to_string(H) + " rows wrong)");

    // (b) System memory: BGR straight into videocrop.
    auto bgr = [](GstVideoFrame& vf, long) {
        auto* d = static_cast<uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&vf, 0));
        const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vf, 0);
        for (int y = 0; y < GST_VIDEO_FRAME_HEIGHT(&vf); ++y)
            for (int x = 0; x < GST_VIDEO_FRAME_WIDTH(&vf); ++x) {
                uint8_t* p = d + static_cast<size_t>(y) * stride + x * 3;
                p[0] = 0; p[1] = static_cast<uint8_t>(y >> 8); p[2] = static_cast<uint8_t>(y & 0xFF);
            }
    };
    const std::vector<uint8_t> b = grabBand(env, false, bgrCaps(1456, 1088, 30), "", bgr, "L10 sysmem");
    const int badB = badRows(b, W, H, top);
    check(!b.empty() && badB == 0, "L10 system-memory path: band rows carry source rows exactly (" +
          std::to_string(badB) + " of " + std::to_string(H) + " rows wrong)");

    // (c) NV12 as the camera delivers it: luma markers on the rows either
    // side of both band edges.  Band row 0 must be source row 544 (bright),
    // band row 319 source row 863 (mid); 543 / 864 must not appear.
    auto i420 = [top, H](GstVideoFrame& vf, long) {
        auto* yp = static_cast<uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&vf, 0));
        const int ys = GST_VIDEO_FRAME_PLANE_STRIDE(&vf, 0);
        for (int y = 0; y < GST_VIDEO_FRAME_HEIGHT(&vf); ++y) {
            const uint8_t v = y == top - 1 ? 60 : y == top ? 200 : y == top + H - 1 ? 120 : y == top + H ? 235 : 16;
            std::memset(yp + static_cast<size_t>(y) * ys, v, static_cast<size_t>(ys));
        }
        for (int pl = 1; pl <= 2; ++pl)
            std::memset(GST_VIDEO_FRAME_PLANE_DATA(&vf, pl), 128,
                        static_cast<size_t>(GST_VIDEO_FRAME_PLANE_STRIDE(&vf, pl)) * ((GST_VIDEO_FRAME_HEIGHT(&vf) + 1) / 2));
    };
    const std::vector<uint8_t> c = grabBand(env, true, "video/x-raw,format=I420,width=1456,height=1088,framerate=30/1",
                                            "nvvidconv ! video/x-raw(memory:NVMM),format=NV12", i420, "L10 NV12");
    if (c.size() != static_cast<size_t>(W) * H * 3) {
        check(false, "L10 NV12 path delivered a full band");
        return;
    }
    auto rowMean = [&](int y) {
        double s = 0;
        for (int x = 0; x < W; ++x) s += c[(static_cast<size_t>(y) * W + x) * 3 + 1];
        return s / W;
    };
    const double r0 = rowMean(0), r1 = rowMean(1), r318 = rowMean(H - 2), r319 = rowMean(H - 1);
    check(r0 > 150 && r1 < 40 && r318 < 40 && r319 > 60 && r319 < 170,
          "L10 NV12 path: band row 0 = source row 544 (" + f(r0, 0) + "), row 319 = 863 (" + f(r319, 0) +
          "), neighbours dark (" + f(r1, 0) + ", " + f(r318, 0) + ")");
}

void testL12ThrowingLogCallback(const TestEnv& env) {
    // Run in a re-exec'd child: if the bug is present, std::terminate takes
    // the child down, not this process.
    const pid_t pid = ::fork();
    if (pid == 0) {
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) { ::dup2(devnull, 1); ::dup2(devnull, 2); }
        const char* args[] = { "lane_test", "--child-b19", env.engine.c_str(), nullptr };
        ::execv("/proc/self/exe", const_cast<char* const*>(args));
        ::_exit(127);
    }
    int status = 0;
    const bool waited = pid > 0 && ::waitpid(pid, &status, 0) == pid;
    const std::string how = !waited ? "not started" :
        WIFEXITED(status) ? "exit " + std::to_string(WEXITSTATUS(status)) :
        WIFSIGNALED(status) ? std::string("killed by ") + strsignal(WTERMSIG(status)) : "?";
    bugCheck("B19", waited && WIFEXITED(status) && WEXITSTATUS(status) == 0,
             "L12 a log callback that throws on the inference thread does not end the process (child: " + how + ")");
}

void testL13Timestamps(const TestEnv& env) {
    LogSink sink;
    LaneDetectorConfig cfg = smallCfg(env.engine, sink, 0, 20);
    cfg.collectTimings = true;
    auto det = makeDetector(640, 480, cfg, "L13");
    if (!det) return;
    Feed feed;
    if (!feed.build(bgrCaps(640, 480, 20), "", det->createBin(), /*doTimestamp=*/true) || !feed.play()) return;
    det->start();
    feed.startFeeding(0.0, fillGrey, /*exactPts=*/false);
    waitFor([&] { return det->processedFrameCount() >= 5; }, 5000);
    int n = 0, ordered = 0, known = 0, quick = 0;
    std::vector<double> capPull, pullDone;
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        const LaneTiming t = det->poll().timing;
        ++n;
        known += t.captureMonoNs >= 0;
        ordered += t.captureMonoNs >= 0 && t.captureMonoNs <= t.pullMonoNs && t.pullMonoNs <= t.doneMonoNs;
        quick += t.captureMonoNs >= 0 && t.pullMonoNs - t.captureMonoNs < 100000000;
        if (t.captureMonoNs >= 0) capPull.push_back((t.pullMonoNs - t.captureMonoNs) / 1e6);
        pullDone.push_back((t.doneMonoNs - t.pullMonoNs) / 1e6);
    }
    feed.toNull();
    det->stop();
    check(known == n, "L13 captureMonoNs known on a live system-clock pipeline (" + std::to_string(known) + "/" + std::to_string(n) + ")");
    check(ordered == n, "L13 capture <= pull <= done (" + std::to_string(ordered) + "/" + std::to_string(n) + ")");
    check(quick == n, "L13 pull - capture < 100 ms when idle (" + std::to_string(quick) + "/" + std::to_string(n) + ")");
    auto med = [](std::vector<double> v) { if (v.empty()) return -1.0; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
    note("L13 medians: capture->pull " + f(med(capPull), 2) + " ms, pull->done " + f(med(pullDone), 2) + " ms");
}

void testL14DualInstance(const TestEnv& env) {
    LogSink s1, s2;
    const long avail0 = memAvailableKb();
    auto a = makeDetector(640, 480, smallCfg(env.engine, s1), "L14 a");
    const long avail1 = memAvailableKb();
    auto b = makeDetector(640, 480, smallCfg(env.engine, s2), "L14 b");
    const long avail2 = memAvailableKb();
    if (!a || !b) return;
    note("L14 MemAvailable: first detector -" + f((avail0 - avail1) / 1024.0, 0) + " MiB, second -" +
         f((avail1 - avail2) / 1024.0, 0) + " MiB");
    const std::vector<uint8_t> fr = branchFrame(*a, 51u);
    LaneResult ra, rb;
    check(a->processFrame(fr.data(), fr.size(), 0, ra) && b->processFrame(fr.data(), fr.size(), 0, rb) &&
              sameResult(ra, rb),
          "L14 two detectors in one process: same frame, identical results");
    Feed fa, fb;
    const bool ok = fa.build(bgrCaps(640, 480, 30), "", a->createBin()) && fa.play() &&
                    fb.build(bgrCaps(640, 480, 30), "", b->createBin()) && fb.play();
    if (!ok) return;
    const uint64_t na = a->processedFrameCount(), nb = b->processedFrameCount();
    a->start();
    b->start();
    fa.startFeeding(0.0, fillGrey);
    fb.startFeeding(0.0, fillGrey);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    const uint64_t da = a->processedFrameCount() - na, db = b->processedFrameCount() - nb;
    fa.toNull();
    fb.toNull();
    a->stop();
    b->stop();
    check(da > 10 && db > 10, "L14 both inference threads run concurrently (" + std::to_string(da) + " and " +
          std::to_string(db) + " results in 3 s at 30 fps)");
}

// B16: --video through processFrame is deterministic.
void testS2VideoDeterminism(const TestEnv&) {
    std::string clip;
    for (const char* c : { "/ufld/example.mp4",
                           "/home/jetson/Documents/github_repos/Ultra-Fast-Lane-Detection-v2/example.mp4" })
        if (fileExists(c)) { clip = c; break; }
    if (clip.empty()) {
        note("S2 skipped: UFLD example.mp4 not found (mount it at /ufld)");
        return;
    }
    const std::string engine = "models/culane_res18_fp16.engine";
    std::vector<LaneResult> r1, r2;
    const bool ok = lanetest::runVideoResults(engine, clip, 1280, 720, 8, 3, r1) &&
                    lanetest::runVideoResults(engine, clip, 1280, 720, 8, 3, r2);
    bool same = ok && !r1.empty() && r1.size() == r2.size();
    for (size_t i = 0; same && i < r1.size(); ++i) same = sameResult(r1[i], r2[i]);
    int lanes = 0;
    for (const auto& r : r1) lanes += r.numLanes >= 1;
    check(same, "S2 --video twice over the first 8 s (every 3rd frame): " + std::to_string(r1.size()) +
          " results, identical run to run (" + std::to_string(lanes) + " with lanes)");
}

} // namespace

// Child for L12 (re-exec'd as "lane_test --child-b19 <engine>"): a running
// detector whose log callback throws, then three logged errors on the
// inference thread.  Exit 0 = survived.
int lanetest::childB19(const std::string& engine) {
    LogSink sink;
    auto det = makeDetector(640, 480, smallCfg(engine, sink), "child");
    if (!det) return 3;
    Feed feed;
    if (!feed.build(bgrCaps(640, 480, 30), "", det->createBin()) || !feed.play()) return 4;
    det->setLogCallback([](dashcam::log::LogLevel, const std::string&) {
        throw std::runtime_error("log sink failed");
    });
    det->start();
    feed.startFeeding(0.0, fillGrey);
    waitFor([&] { return det->processedFrameCount() >= 2; }, 5000);
    LaneDetectorTestHook::injectFault(*det, LaneDetectorTestHook::Fault::Enqueue, 3);
    waitFor([&] { return det->stats().trtFail >= 3; }, 5000);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    feed.toNull();
    det->stop();
    return det->stats().trtFail >= 3 ? 0 : 5;
}

namespace {
} // namespace

namespace lanetest {

std::vector<TestCase> gpuTests() {
    return {
        { "P1 preprocess vs reference",   30, testP1Reference },
        { "P2 channel order + CHW",       20, testP2Channels },
        { "P3 borders",                   20, testP3Borders },
        { "P4 G2 training ROI",           20, testP4TrainingRoi },
        { "P5 1280x212 upsample",         20, testP5VerticalUpsample },
        { "E1 engine IO contract",        60, testE1IoContract },
        { "E2 fp16 IO rejected",          60, testE2Fp16Io },
        { "E3 load cost",                 90, testE3LoadCost },
        { "E4 bit-identical rerun",       60, testE4BitIdentical },
        { "E5 engine identity",           30, testE5Identity },
        { "PF1 determinism + timings",    60, testPF1Determinism },
        { "PF2 refused while running",    60, testPF2RefusedWhileRunning },
        { "PF3 result sink",              60, testPF3ResultSink },
        { "T1 tiny engine decode",        30, testT1TinyDecode },
        { "T2 runModelSized",             30, testT2RunModelSized },
        { "L1 createBin while running",   30, testL1CreateBinWhileRunning },
        { "L2 spin after stop / EOS",     40, testL2SpinAfterStop },
        { "L3 rate gate",                 90, testL3RateGate },
        { "L4 restart state",             30, testL4RestartState },
        { "L5 NVMM size mismatch",        30, testL5NvmmSizeMismatch },
        { "L6 stride-padded width",       30, testL6StridePadded },
        { "L7 error flood",               40, testL7ErrorFlood },
        { "L8 log callback while running", 30, testL8LogCallbackWhileRunning },
        { "L9 50 restart cycles",        150, testL9RestartCycles },
        { "L10 golden rows",              60, testL10GoldenRows },
        { "L12 throwing log callback",    60, testL12ThrowingLogCallback },
        { "L13 timestamps",               30, testL13Timestamps },
        { "L14 dual instance",            60, testL14DualInstance },
        { "S2 video determinism",        120, testS2VideoDeterminism },
    };
}

} // namespace lanetest
