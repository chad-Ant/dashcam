// timesync_test — validates libtimesync (NTP first, GPS fallback) against a
// simulated wall clock and monotonic clock: nothing touches the real clock, so
// it needs no privileges.
//
// Covers: UTC→Unix conversion and invalid dates; NTP small offset (confirm,
// no step) vs large offset (step); an NTP sample applied after another
// correction (GPS / host) is applied once, never twice; GPS needs its time-valid flag, a sane year,
// second BOUNDARIES and N agreeing boundaries before stepping; a skipped or
// backwards second restarts confirmation; GPS may not override a fresh NTP
// result but may once NTP has been silent past its authority window; a failing
// clock set is reported, not assumed; ClockJumpDetector sees steps, not drift.
//
// Usage: timesync_test

#include "libtimesync.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace dashcam::timesync;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

// Simulated machine: a monotonic clock and a wall clock that is `wallOffset`
// ahead of it.  Setting the clock changes wallOffset, as clock_settime would.
struct Sim {
    double mono = 1000.0;
    double wallOffset = 0.0;
    int    sets = 0;
    bool   setFails = false;
    double wall() const { return mono + wallOffset; }
    TimeKeeper keeper(const ClockPolicy& p = {}) {
        return TimeKeeper(p, {}, [this] { return wall(); }, [this] { return mono; },
                          [this](double t, std::string& err) {
                              if (setFails) { err = "simulated EPERM"; return false; }
                              wallOffset = t - mono;
                              ++sets;
                              return true;
                          });
    }
};

static UtcFields utcOf(int64_t unixSec) {
    const time_t t = static_cast<time_t>(unixSec);
    struct tm g;
    gmtime_r(&t, &g);
    return {g.tm_year + 1900, g.tm_mon + 1, g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec};
}

// Feed GPS the way the bridge does: 10 Hz samples whose UTC second follows TRUE
// time `truth(mono)`; returns whether any sample stepped the clock.
template <typename Truth>
static bool feedGps(Sim& sim, TimeKeeper& k, Truth truth, double seconds, bool valid = true) {
    bool stepped = false;
    for (int i = 0; i < int(seconds * 10); ++i) {
        sim.mono += 0.1;
        const int64_t sec = static_cast<int64_t>(std::floor(truth(sim.mono)));
        stepped = k.offerGps(utcOf(sec), valid) || stepped;
    }
    return stepped;
}

int main() {
    std::printf("--- UTC conversion ---\n");
    int64_t u = 0;
    check(utcToUnix({2000, 1, 1, 0, 0, 0}, u) && u == 946684800, "2000-01-01 00:00:00 = 946684800");
    check(utcToUnix({2024, 2, 29, 12, 0, 0}, u) && u == 1709208000, "2024-02-29 12:00:00 (leap day)");
    check(utcToUnix({2026, 9, 24, 23, 59, 60}, u) && u == 1790294399, "leap second :60 read as :59");
    check(!utcToUnix({2026, 2, 30, 0, 0, 0}, u), "30 February rejected");
    check(!utcToUnix({2026, 13, 1, 0, 0, 0}, u), "month 13 rejected");
    check(!utcToUnix({2026, 1, 1, 24, 0, 0}, u), "hour 24 rejected");
    check(!utcToUnix({0, 0, 0, 0, 0, 0}, u), "all-zero (receiver without a date) rejected");

    // "Now" for the scenarios: 2026-09-24 04:42:35 UTC.  The Jetson booted with
    // the last shutdown time, ~17 h behind.
    const double truthEpoch = 1790224955.0;
    const double bootError  = -17 * 3600.0;
    // True UTC at a simulated monotonic instant (every Sim starts at mono 1000).
    auto truthAt = [&](double mono) { return truthEpoch + (mono - 1000.0); };

    std::printf("--- NTP ---\n");
    {
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono + 0.4;       // 0.4 s off: leave it to timesyncd
        TimeKeeper k = sim.keeper();
        check(!k.offerNtp(truthAt(sim.mono), sim.mono, "pool") && sim.sets == 0 &&
              k.lastSource() == Source::Ntp, "0.4 s offset: confirmed, not stepped");
    }
    {
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono + bootError;
        TimeKeeper k = sim.keeper();
        check(k.offerNtp(truthAt(sim.mono), sim.mono, "pool") && sim.sets == 1, "17 h behind: stepped");
        check(std::fabs(sim.wall() - truthAt(sim.mono)) < 1e-6, "clock now correct");
        check(!k.offerNtp(NAN, sim.mono, "pool") && sim.sets == 1, "NaN sample ignored");
        check(!k.offerNtp(truthAt(sim.mono), sim.mono + 5, "pool") && sim.sets == 1,
              "sample from the future ignored");
    }
    {
        Sim sim;
        sim.setFails = true;
        TimeKeeper k = sim.keeper();
        check(!k.offerNtp(truthAt(sim.mono) + 3600, sim.mono, "pool") && k.lastSource() == Source::None,
              "failing clock set: reported, source not claimed");
    }

    std::printf("--- NTP sample vs a concurrent correction (applied once, never twice) ---\n");
    {
        // Measured while 17 h behind; before it is applied, GPS or the host's own
        // NTP service corrects the clock.  An offset would now add 17 h again.
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono + bootError;
        TimeKeeper k = sim.keeper();
        const double utcRx = truthAt(sim.mono), monoRx = sim.mono;
        sim.wallOffset = truthEpoch - 1000.0;                 // someone else fixed the clock
        sim.mono += 2.0;
        check(!k.offerNtp(utcRx, monoRx, "pool") && sim.sets == 0,
              "late sample after an external fix: no step");
        check(std::fabs(sim.wall() - truthAt(sim.mono)) < 1e-6, "clock still correct (no +17 h)");
    }
    {
        // A sample that waited 30 s (clock still wrong) lands on the time NOW,
        // not the time it was taken.
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono + bootError;
        TimeKeeper k = sim.keeper();
        const double utcRx = truthAt(sim.mono), monoRx = sim.mono;
        sim.mono += 30.0;
        check(k.offerNtp(utcRx, monoRx, "pool") && std::fabs(sim.wall() - truthAt(sim.mono)) < 1e-6,
              "30 s old sample: stepped to the current true time");
        check(!k.offerNtp(utcRx, monoRx, "pool") && sim.sets == 1,
              "same sample offered twice: applied once");
    }
    {
        // GPS steps the clock between an NTP measurement and its application.
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono + bootError;
        TimeKeeper k = sim.keeper();
        const double utcRx = truthAt(sim.mono), monoRx = sim.mono;
        feedGps(sim, k, truthAt, 5);                          // GPS fixes the clock first
        check(sim.sets == 1 && k.lastSource() == Source::Gps, "GPS corrected the clock first");
        check(!k.offerNtp(utcRx, monoRx, "pool") && sim.sets == 1 &&
              std::fabs(sim.wall() - truthAt(sim.mono)) < 0.2,
              "then the stale NTP sample: confirms, no second step");
    }

    std::printf("--- GPS fallback (no NTP) ---\n");
    {
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono + bootError;
        const double truthOff = truthEpoch - sim.mono;       // true wall − mono
        auto truth = [&](double mono) { return mono + truthOff; };
        TimeKeeper k = sim.keeper();
        check(!feedGps(sim, k, truth, 5, /*valid=*/false) && sim.sets == 0, "time-valid flag clear: ignored");
        check(!k.offerGps({2000, 1, 1, 0, 0, 0}, true) && sim.sets == 0, "year 2000 (unset receiver): ignored");
        check(!feedGps(sim, k, truth, 1.05) && sim.sets == 0, "one boundary: not trusted yet");
        const bool stepped = feedGps(sim, k, truth, 3);
        check(stepped && sim.sets == 1 && k.lastSource() == Source::Gps, "3 agreeing boundaries: stepped");
        const double err = sim.wall() - truth(sim.mono);
        check(std::fabs(err) < 0.2, "clock within 0.2 s of true time (" + std::to_string(err) + " s)");
        check(!feedGps(sim, k, truth, 10) && sim.sets == 1, "keeps agreeing: no further steps");
    }
    {
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono + bootError;
        TimeKeeper k = sim.keeper();
        // Receiver glitch: the second jumps around; every jump restarts confirmation.
        double glitchOff = truthEpoch - sim.mono;
        int n = 0;
        auto glitchy = [&](double mono) { if (++n % 25 == 0) glitchOff += 7; return mono + glitchOff; };
        check(!feedGps(sim, k, glitchy, 10) && sim.sets == 0, "second skipping every 2.5 s: never trusted");
    }

    std::printf("--- NTP authority over GPS ---\n");
    {
        Sim sim;
        sim.wallOffset = truthEpoch - sim.mono;              // correct, NTP just confirmed it
        TimeKeeper k = sim.keeper();
        k.offerNtp(truthAt(sim.mono), sim.mono, "pool");
        const double badOff = truthEpoch - sim.mono + 30;    // GPS 30 s wrong
        auto bad = [&](double mono) { return mono + badOff; };
        check(!feedGps(sim, k, bad, 5) && sim.sets == 0, "fresh NTP: GPS 30 s off is not applied");
        sim.mono += 3600;                                    // NTP silent for an hour (offline)
        check(feedGps(sim, k, bad, 5) && sim.sets == 1 && k.lastSource() == Source::Gps,
              "NTP silent past 1 h: GPS allowed to step");
    }

    std::printf("--- clock jump detector ---\n");
    {
        Sim sim;
        ClockJumpDetector d(2.0, [&] { return sim.wall(); }, [&] { return sim.mono; });
        double j = 0;
        sim.mono += 1;
        check(!d.poll(j), "no change: no jump");
        sim.wallOffset += 0.05;                              // slew by timesyncd
        sim.mono += 1;
        check(!d.poll(j), "50 ms slew: not a jump");
        sim.wallOffset += 17 * 3600.0;                       // stepped by NTP/GPS
        sim.mono += 0.2;
        check(d.poll(j) && std::fabs(j - 61200.0) < 1e-6, "17 h step: detected with its size");
        check(!d.poll(j), "reported once");
    }

    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails,
                g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
