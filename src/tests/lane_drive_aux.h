// lane_drive_aux.h — the GStreamer-free parts of lane_drive (plan P5b,
// ~/drive_logs/lane_test_suite_plan_20261025.md sections 3.3, 3.4, 9.2, 12.4),
// kept here so lane_test --unit can test them without a camera:
//
//   JpegRing      in-RAM ring of encoded camera frames, capped in bytes (clip
//                 ring; also the source of exact-PTS dump frames)
//   ClipBudget    token bucket that rations kept clips by priority
//   ClipPlanner   turns triggers into clip windows (merge, extend, collect
//                 frames from the ring as they arrive)
//   TensorRing    the last N output-tensor sets (event dumps)
//   MountMonitor  edge-based view-shift estimate against a parked reference
//                 (gradient NCC over a car-fixed region, 1 Hz, 5 s median)
//
// Times: mono = CLOCK_MONOTONIC ns, pts = the camera pipeline's buffer PTS.

#ifndef LANE_DRIVE_AUX_H
#define LANE_DRIVE_AUX_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lanedrive {

// ─── clip ring ───────────────────────────────────────────────────────────────

struct JFrame {
    int64_t pts = -1;
    int64_t mono = -1;                                   ///< capture time on CLOCK_MONOTONIC (−1 unknown)
    std::shared_ptr<const std::vector<uint8_t>> data;    ///< JPEG bytes (shared with clip jobs)
};

/// Frames are copied in (a camera's buffer pool must not be held) and shared
/// out: a clip job keeps its frames alive after the ring has dropped them.
class JpegRing {
public:
    explicit JpegRing(size_t capBytes) : cap_(capBytes) {}

    void setEnabled(bool on) {
        std::lock_guard<std::mutex> lk(m_);
        enabled_ = on;
        if (!on) { q_.clear(); bytes_ = 0; }
    }
    bool enabled() const { std::lock_guard<std::mutex> lk(m_); return enabled_; }

    void push(int64_t pts, int64_t mono, const uint8_t* d, size_t n) {
        auto buf = std::make_shared<std::vector<uint8_t>>(d, d + n);
        std::lock_guard<std::mutex> lk(m_);
        if (!enabled_) return;
        q_.push_back(JFrame{ pts, mono, std::move(buf) });
        bytes_ += n;
        ++pushed_;
        totalBytes_ += n;
        while (bytes_ > cap_ && q_.size() > 1) {
            bytes_ -= q_.front().data->size();
            q_.pop_front();
        }
    }
    /// Frames with mono in (afterMono, toMono], oldest first.
    std::vector<JFrame> rangeMono(int64_t afterMono, int64_t toMono) const {
        std::vector<JFrame> out;
        std::lock_guard<std::mutex> lk(m_);
        for (const auto& f : q_)
            if (f.mono > afterMono && f.mono <= toMono) out.push_back(f);
        return out;
    }
    /// The frame with exactly @p pts; else, with @p nearest, the closest one
    /// (|dpts| reported).  False when the ring is empty / no exact match.
    bool find(int64_t pts, JFrame& out, int64_t& dpts, bool nearest) const {
        std::lock_guard<std::mutex> lk(m_);
        const JFrame* best = nullptr;
        int64_t bestD = INT64_MAX;
        for (const auto& f : q_) {
            const int64_t d = f.pts > pts ? f.pts - pts : pts - f.pts;
            if (d < bestD) { bestD = d; best = &f; }
        }
        if (!best || (bestD != 0 && !nearest)) return false;
        out = *best;
        dpts = best->pts - pts;
        return true;
    }
    int64_t newestMono() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.empty() ? -1 : q_.back().mono;
    }
    /// Seconds of video held (newest − oldest mono).
    double spanS() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size() < 2 ? 0.0 : (q_.back().mono - q_.front().mono) / 1e9;
    }
    size_t bytes() const { std::lock_guard<std::mutex> lk(m_); return bytes_; }
    uint64_t pushed() const { std::lock_guard<std::mutex> lk(m_); return pushed_; }
    uint64_t totalBytes() const { std::lock_guard<std::mutex> lk(m_); return totalBytes_; }
    size_t capBytes() const { return cap_; }

private:
    const size_t cap_;
    mutable std::mutex m_;
    std::deque<JFrame> q_;
    size_t bytes_ = 0;
    uint64_t pushed_ = 0, totalBytes_ = 0;
    bool enabled_ = true;
};

// ─── clip budget ─────────────────────────────────────────────────────────────

/// Priorities, most important first (plan 12.4: faults → LC → stop→go → rest).
enum ClipPriority : int { kFault = 0, kLaneChange = 1, kStopGo = 2, kRest = 3 };

/// Token bucket in bytes.  Refill = budget GB/h; capacity = capacityS of
/// refill.  A clip of priority p is admitted only if the bucket keeps at
/// least reserve[p] × capacity after paying the estimate, so low-priority
/// clips stop first when the budget binds and faults are kept longest.
class ClipBudget {
public:
    ClipBudget(double gbPerHour, double capacityS = 600, double startFrac = 0.5)
        : rate_(gbPerHour * 1e9 / 3600.0), cap_(rate_ * capacityS), tokens_(cap_ * startFrac) {}

    bool admit(int priority, double estBytes, double nowS) {
        refill(nowS);
        static const double reserve[4] = { 0.0, 0.25, 0.5, 0.75 };
        const double keep = reserve[std::max(0, std::min(3, priority))] * cap_;
        if (tokens_ - estBytes < keep) return false;
        tokens_ -= estBytes;
        return true;
    }
    /// Settle a clip: @p delta = actual − estimated bytes (may be negative).
    void adjust(double delta) { tokens_ = std::min(cap_, tokens_ - delta); }
    double tokens(double nowS) { refill(nowS); return tokens_; }
    double capacity() const { return cap_; }

private:
    void refill(double nowS) {
        if (last_ >= 0 && nowS > last_) tokens_ = std::min(cap_, tokens_ + (nowS - last_) * rate_);
        last_ = std::max(last_, nowS);
    }
    double rate_, cap_, tokens_;
    double last_ = -1;
};

// ─── clip planner ────────────────────────────────────────────────────────────

struct ClipJob {
    int id = 0;
    std::string triggers;            ///< every trigger merged into the window, ';'-separated
    int priority = kRest;
    int64_t eventMono = 0;           ///< the first trigger
    int64_t startMono = 0, endMono = 0;
    int64_t lastMono = INT64_MIN;    ///< newest frame collected
    double estBytes = 0;
    std::vector<JFrame> frames;
    size_t bytes = 0;
    bool overflow = false;           ///< the pending-bytes cap cut frames
};

/// Triggers → clip windows [t − pre, t + post].  A trigger inside (or
/// touching) the open window extends it, up to maxLen; otherwise it asks the
/// budget for a new clip.  collect() moves the ring's new frames into the open
/// jobs; finished() hands back the jobs whose window has been covered (or
/// timed out — a stalled camera must not hold a job forever).
class ClipPlanner {
public:
    struct Params {
        int64_t preNs = 10'000'000'000LL, postNs = 10'000'000'000LL;
        int64_t maxLenNs = 60'000'000'000LL;
        int64_t timeoutNs = 5'000'000'000LL;     ///< past endMono with no newer frame
        size_t pendingCap = 512u << 20;          ///< bytes held by open jobs
    };
    ClipPlanner() = default;
    explicit ClipPlanner(Params p) : p_(p) {}

    /// @return the job id the trigger went into, 0 = refused by the budget,
    ///         −1 = clips disabled.
    int trigger(const std::string& name, int priority, int64_t mono, double bytesPerSec,
                ClipBudget& budget, bool enabled = true) {
        if (!enabled) return -1;
        for (auto& j : open_) {
            if (mono - p_.preNs <= j.endMono && mono >= j.startMono) {
                j.endMono = std::min(j.startMono + p_.maxLenNs, std::max(j.endMono, mono + p_.postNs));
                if (j.triggers.size() < 400) j.triggers += ";" + name;
                j.priority = std::min(j.priority, priority);
                return j.id;
            }
        }
        ClipJob j;
        j.startMono = mono - p_.preNs;
        j.endMono = mono + p_.postNs;
        j.estBytes = bytesPerSec * (j.endMono - j.startMono) / 1e9;
        if (!budget.admit(priority, j.estBytes, mono / 1e9)) { ++refused_; return 0; }
        j.id = ++nextId_;
        j.triggers = name;
        j.priority = priority;
        j.eventMono = mono;
        open_.push_back(std::move(j));
        return open_.back().id;
    }
    void collect(const JpegRing& ring) {
        for (auto& j : open_) {
            const int64_t after = std::max(j.lastMono, j.startMono - 1);
            for (auto& f : ring.rangeMono(after, j.endMono)) {
                if (pending_ + f.data->size() > p_.pendingCap) { j.overflow = true; ++overflowFrames_; continue; }
                pending_ += f.data->size();
                j.bytes += f.data->size();
                j.lastMono = f.mono;
                j.frames.push_back(std::move(f));
            }
        }
    }
    /// Jobs complete at @p nowMono (newest frame past the window, or timeout).
    std::vector<ClipJob> finished(int64_t newestMono, int64_t nowMono, bool all = false) {
        std::vector<ClipJob> done;
        for (auto it = open_.begin(); it != open_.end();) {
            if (all || newestMono >= it->endMono || nowMono > it->endMono + p_.timeoutNs) {
                pending_ -= std::min(pending_, it->bytes);
                done.push_back(std::move(*it));
                it = open_.erase(it);
            } else {
                ++it;
            }
        }
        return done;
    }
    void dropAll() { open_.clear(); pending_ = 0; }
    size_t openJobs() const { return open_.size(); }
    size_t pendingBytes() const { return pending_; }
    uint64_t refused() const { return refused_; }
    uint64_t overflowFrames() const { return overflowFrames_; }

private:
    Params p_;
    std::deque<ClipJob> open_;
    int nextId_ = 0;
    size_t pending_ = 0;
    uint64_t refused_ = 0, overflowFrames_ = 0;
};

// ─── tensor ring ─────────────────────────────────────────────────────────────

struct TensorSet {
    uint64_t seq = 0;
    int64_t pts = -1, captureMono = -1;
    uint32_t counts[4] = {};
    std::vector<float> data;          ///< the four outputs back to back
};

class TensorRing {
public:
    explicit TensorRing(size_t slots) : slots_(slots) {}
    void put(uint64_t seq, int64_t pts, int64_t mono, const float* const outs[4], const size_t counts[4]) {
        std::lock_guard<std::mutex> lk(m_);
        TensorSet& t = slots_[next_ % slots_.size()];
        ++next_;
        t.seq = seq; t.pts = pts; t.captureMono = mono;
        size_t total = 0;
        for (int i = 0; i < 4; ++i) { t.counts[i] = static_cast<uint32_t>(counts[i]); total += counts[i]; }
        t.data.resize(total);
        size_t o = 0;
        for (int i = 0; i < 4; ++i) {
            std::memcpy(t.data.data() + o, outs[i], counts[i] * sizeof(float));
            o += counts[i];
        }
    }
    /// Copies of every filled slot, oldest first.
    std::vector<TensorSet> snapshot() const {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<TensorSet> out;
        const size_t n = std::min<size_t>(next_, slots_.size());
        for (size_t i = 0; i < n; ++i) out.push_back(slots_[(next_ - n + i) % slots_.size()]);
        return out;
    }
private:
    mutable std::mutex m_;
    std::vector<TensorSet> slots_;
    uint64_t next_ = 0;
};

// ─── mount monitor ───────────────────────────────────────────────────────────

struct Gray {
    int w = 0, h = 0;
    std::vector<uint8_t> px;
    uint8_t at(int x, int y) const { return px[static_cast<size_t>(y) * w + x]; }
};

/// Pixel-wise median of same-sized frames (a parked reference from a few
/// seconds of frames: passing cars and noise drop out).
inline Gray medianOf(const std::vector<Gray>& v) {
    Gray out;
    if (v.empty()) return out;
    out.w = v[0].w; out.h = v[0].h;
    out.px.resize(v[0].px.size());
    std::vector<uint8_t> s(v.size());
    for (size_t i = 0; i < out.px.size(); ++i) {
        for (size_t k = 0; k < v.size(); ++k) s[k] = v[k].px[i];
        std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
        out.px[i] = s[s.size() / 2];
    }
    return out;
}

/// Region of the frame that is fixed to the car (hood, dashboard, A-pillar),
/// as fractions of the frame.
struct MountRoi { float y0 = 0.75f, y1 = 1.0f, x0 = 0.05f, x1 = 0.95f; };

struct MountSample {
    bool ok = false;
    float dx = 0, dy = 0, rollDeg = 0;   ///< source px (scaled), degrees
    float conf = 0;                      ///< peak normalised cross-correlation
    /// The region still shows edges (gradient std >= lostFeatureRatio of the
    /// reference's) but nothing in the search range matches: the view moved
    /// further than the search reaches (10-03: ~230 px and ~9 deg) — a shift,
    /// not a dark frame.
    bool lost = false;
    float featureRatio = 0;              ///< region gradient std / reference's
};

class MountMonitor {
public:
    struct Params {
        int searchPx = 12;           ///< ± search at tap resolution
        float minConf = 0.5f;        ///< below: no measurement
        float minStd = 2.0f;         ///< template gradient std below: too dark / featureless
        float lostConf = 0.35f;      ///< peak below this with features present = lost view
        float lostFeatureRatio = 0.5f;
        float shiftPx = 6.0f;        ///< |5 s median| above (source px) ...
        float rollDeg = 0.5f;        ///< ... or roll above ...
        int64_t sustainNs = 5'000'000'000LL;   ///< ... for this long = FAIL (plan V2)
    };
    MountMonitor() = default;
    explicit MountMonitor(Params p) : p_(p) {}

    /// @p scale = source px per tap px.
    bool setReference(const Gray& ref, const MountRoi& roi, float scale) {
        if (ref.w < 32 || ref.h < 32) return false;
        G0_ = gradient(ref);
        w_ = ref.w; h_ = ref.h; scale_ = scale;
        const int R = p_.searchPx;
        r0_ = std::max(R + 1, static_cast<int>(roi.y0 * h_));
        r1_ = std::min(h_ - R - 1, static_cast<int>(roi.y1 * h_));
        c0_ = std::max(R + 1, static_cast<int>(roi.x0 * w_));
        c1_ = std::min(w_ - R - 1, static_cast<int>(roi.x1 * w_));
        if (r1_ - r0_ < 8 || c1_ - c0_ < 32) return false;
        T_.assign(static_cast<size_t>(r1_ - r0_) * (c1_ - c0_), 0.0f);
        double sum = 0, sq = 0;
        for (int y = r0_; y < r1_; ++y)
            for (int x = c0_; x < c1_; ++x) { const float g = G0_[idx(x, y)]; sum += g; sq += g * g; }
        const double n = static_cast<double>(T_.size());
        const double mean = sum / n;
        tStd_ = static_cast<float>(std::sqrt(std::max(0.0, sq / n - mean * mean)));
        for (int y = r0_; y < r1_; ++y)
            for (int x = c0_; x < c1_; ++x)
                T_[static_cast<size_t>(y - r0_) * (c1_ - c0_) + (x - c0_)] =
                    tStd_ > 0 ? static_cast<float>((G0_[idx(x, y)] - mean) / tStd_) : 0.0f;
        haveRef_ = true;
        return true;
    }
    bool hasReference() const { return haveRef_; }
    float referenceStd() const { return tStd_; }

    MountSample measure(const Gray& g) const {
        MountSample s;
        if (!haveRef_ || g.w != w_ || g.h != h_ || tStd_ < p_.minStd) return s;
        const std::vector<float> G = gradient(g);
        const int R = p_.searchPx;
        {   // features in the (unshifted) region
            double sum = 0, sq = 0;
            for (int y = r0_; y < r1_; ++y)
                for (int x = c0_; x < c1_; ++x) { const float v = G[idx(x, y)]; sum += v; sq += v * v; }
            const double n = static_cast<double>(r1_ - r0_) * (c1_ - c0_);
            const double sd = std::sqrt(std::max(0.0, sq / n - (sum / n) * (sum / n)));
            s.featureRatio = tStd_ > 0 ? static_cast<float>(sd / tStd_) : 0.0f;
        }
        // NCC over the whole region, every integer shift.
        std::vector<float> ncc(static_cast<size_t>(2 * R + 1) * (2 * R + 1), -2.0f);
        int bx = 0, by = 0;
        float best = -2.0f;
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                const float v = corr(G, dx, dy, c0_, c1_);
                ncc[static_cast<size_t>(dy + R) * (2 * R + 1) + (dx + R)] = v;
                if (v > best) { best = v; bx = dx; by = dy; }
            }
        auto at = [&](int dx, int dy) { return ncc[static_cast<size_t>(dy + R) * (2 * R + 1) + (dx + R)]; };
        float fx = static_cast<float>(bx), fy = static_cast<float>(by);
        if (bx > -R && bx < R) fx += parabola(at(bx - 1, by), best, at(bx + 1, by));
        if (by > -R && by < R) fy += parabola(at(bx, by - 1), best, at(bx, by + 1));
        // Roll: the vertical shift of the left and right halves.
        const int mid = (c0_ + c1_) / 2;
        const float yl = subShiftY(G, bx, by, c0_, mid), yr = subShiftY(G, bx, by, mid, c1_);
        const float dxHalves = 0.5f * (c1_ - c0_);
        s.conf = best;
        s.ok = best >= p_.minConf;
        s.lost = !s.ok && best < p_.lostConf && s.featureRatio >= p_.lostFeatureRatio;
        s.dx = fx * scale_;
        s.dy = fy * scale_;
        s.rollDeg = static_cast<float>(std::atan2(yr - yl, dxHalves) * 180.0 / M_PI);
        return s;
    }

    struct Verdict {
        bool have = false;               ///< a 5 s median exists
        float med5Dx = 0, med5Dy = 0, med5Roll = 0;
        bool exceeding = false;          ///< the median is over a threshold now, or the view is lost
        bool shifted = false;            ///< latched: sustained ≥ sustainNs at least once (FAIL)
        bool onEdge = false, offEdge = false;   ///< this sample started / ended a shift
    };
    /// One 1 Hz sample.  shifted latches (a rigid mount that moved stays a FAIL
    /// for the run); offEdge reports the view coming back.
    /// A sample that is neither measured nor lost (dark, featureless) keeps
    /// the state as it is: darkness never starts or ends a shift.
    Verdict feed(int64_t mono, const MountSample& s) {
        Verdict v;
        v.shifted = shifted_;
        if (s.ok) {
            win_.push_back(s);
            if (win_.size() > 5) win_.pop_front();
        }
        if (!s.ok && !s.lost) return v;
        if (s.lost) win_.clear();            // the old medians describe a view that is gone
        auto med = [&](float MountSample::*f) {
            std::vector<float> a;
            for (const auto& w : win_) a.push_back(w.*f);
            std::nth_element(a.begin(), a.begin() + a.size() / 2, a.end());
            return a[a.size() / 2];
        };
        v.have = !win_.empty();
        if (v.have) {
            v.med5Dx = med(&MountSample::dx);
            v.med5Dy = med(&MountSample::dy);
            v.med5Roll = med(&MountSample::rollDeg);
        }
        v.exceeding = s.lost || (v.have && (std::fabs(v.med5Dx) > p_.shiftPx || std::fabs(v.med5Dy) > p_.shiftPx ||
                                            std::fabs(v.med5Roll) > p_.rollDeg));
        if (v.exceeding) {
            if (exceedSince_ < 0) exceedSince_ = mono;
            clearSince_ = -1;
            if (!active_ && mono - exceedSince_ >= p_.sustainNs) {
                active_ = shifted_ = true;
                v.onEdge = true;
            }
        } else {
            exceedSince_ = -1;
            if (clearSince_ < 0) clearSince_ = mono;
            if (active_ && mono - clearSince_ >= p_.sustainNs) { active_ = false; v.offEdge = true; }
        }
        v.shifted = shifted_;
        return v;
    }
    bool active() const { return active_; }

    /// Gradient magnitude (|gx| + |gy|, central differences) of a 2×2 box-
    /// smoothed image.  Public for tests.
    static std::vector<float> gradient(const Gray& g) {
        std::vector<float> s(g.px.size(), 0.0f), out(g.px.size(), 0.0f);
        for (int y = 0; y + 1 < g.h; ++y)
            for (int x = 0; x + 1 < g.w; ++x)
                s[static_cast<size_t>(y) * g.w + x] =
                    0.25f * (g.at(x, y) + g.at(x + 1, y) + g.at(x, y + 1) + g.at(x + 1, y + 1));
        for (int y = 1; y + 2 < g.h; ++y)
            for (int x = 1; x + 2 < g.w; ++x) {
                const size_t i = static_cast<size_t>(y) * g.w + x;
                out[i] = std::fabs(s[i + 1] - s[i - 1]) + std::fabs(s[i + g.w] - s[i - g.w]);
            }
        return out;
    }

private:
    size_t idx(int x, int y) const { return static_cast<size_t>(y) * w_ + x; }
    // NCC of template columns [ca, cb) against G shifted by (dx, dy).
    float corr(const std::vector<float>& G, int dx, int dy, int ca, int cb) const {
        double st = 0, sw = 0, sw2 = 0;
        const int tw = c1_ - c0_;
        for (int y = r0_; y < r1_; ++y) {
            const float* t = &T_[static_cast<size_t>(y - r0_) * tw];
            const float* row = &G[idx(0, y + dy)];
            for (int x = ca; x < cb; ++x) {
                const float w = row[x + dx];
                st += t[x - c0_] * w;
                sw += w;
                sw2 += static_cast<double>(w) * w;
            }
        }
        const double n = static_cast<double>(r1_ - r0_) * (cb - ca);
        const double var = sw2 / n - (sw / n) * (sw / n);
        if (var <= 1e-6) return -1.0f;
        // T is zero-mean over the full width only; for a half, re-centre.
        double tm = 0;
        if (ca != c0_ || cb != c1_) {
            for (int y = r0_; y < r1_; ++y)
                for (int x = ca; x < cb; ++x) tm += T_[static_cast<size_t>(y - r0_) * tw + (x - c0_)];
            tm /= n;
        }
        return static_cast<float>((st / n - tm * (sw / n)) / std::sqrt(var));
    }
    // Sub-pixel vertical shift of columns [ca, cb) around (bx, by), dy in ±3.
    float subShiftY(const std::vector<float>& G, int bx, int by, int ca, int cb) const {
        const int R = p_.searchPx;
        float best = -2.0f;
        int b = by;
        float v[7];
        for (int k = -3; k <= 3; ++k) {
            const int dy = by + k;
            v[k + 3] = dy >= -R && dy <= R ? corr(G, bx, dy, ca, cb) : -2.0f;
            if (v[k + 3] > best) { best = v[k + 3]; b = dy; }
        }
        const int k = b - by + 3;
        float f = static_cast<float>(b);
        if (k > 0 && k < 6) f += parabola(v[k - 1], v[k], v[k + 1]);
        return f;
    }
    static float parabola(float l, float c, float r) {
        const float den = l - 2 * c + r;
        if (std::fabs(den) < 1e-9f) return 0.0f;
        return std::max(-0.5f, std::min(0.5f, 0.5f * (l - r) / den));
    }

    Params p_;
    bool haveRef_ = false;
    int w_ = 0, h_ = 0, r0_ = 0, r1_ = 0, c0_ = 0, c1_ = 0;
    float scale_ = 1, tStd_ = 0;
    std::vector<float> G0_, T_;
    std::deque<MountSample> win_;
    int64_t exceedSince_ = -1, clearSince_ = -1;
    bool active_ = false, shifted_ = false;
};

} // namespace lanedrive

#endif // LANE_DRIVE_AUX_H
