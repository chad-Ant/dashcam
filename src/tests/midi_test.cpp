/**
 * midi_test — smoke test for libmidi
 *
 * Usage:
 *   ./midi_test [sounds_dir] [device]
 *
 *   sounds_dir  directory containing *.wav files (default: /data/sounds)
 *   device      ALSA PCM device name          (default: default)
 *
 * What it does:
 *   1. Loads all *.wav files from the directory.
 *   2. Plays each sound once (non-looping) and waits for it to finish.
 *   3. Plays the first sound in a loop for 3 seconds, then stops it.
 */

#include "liblog.h"
#include "libmidi.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>

int main(int argc, char* argv[]) {
    const std::string soundsDir = (argc > 1) ? argv[1] : "/data/sounds";
    const std::string device    = (argc > 2) ? argv[2] : "default";

    dashcam::log::init("/tmp");
    auto log = dashcam::log::getCallback();

    dashcam::midi::MidiPlayer player;
    player.setLogCallback(log);

    // ── condition variable so the main thread can wait for STOPPED ────────────
    std::mutex              doneMtx;
    std::condition_variable doneCv;
    std::atomic<bool>       soundDone{false};

    player.setPlaybackCallback(
        [&](const std::string& name, dashcam::midi::PlaybackEvent ev) {
            const char* evStr =
                ev == dashcam::midi::PlaybackEvent::STARTED ? "STARTED" :
                ev == dashcam::midi::PlaybackEvent::STOPPED ? "STOPPED" :
                ev == dashcam::midi::PlaybackEvent::LOOPED  ? "LOOPED"  : "ERROR";
            std::fprintf(stdout, "[midi_test] %-8s  %s\n", evStr, name.c_str());
            std::fflush(stdout);

            if (ev == dashcam::midi::PlaybackEvent::STOPPED ||
                ev == dashcam::midi::PlaybackEvent::ERROR) {
                std::lock_guard<std::mutex> lk(doneMtx);
                soundDone.store(true);
                doneCv.notify_one();
            }
        });

    if (player.loadSoundsFromDirectory(soundsDir) == 0) {
        std::fprintf(stderr, "midi_test: no sounds loaded from '%s'\n", soundsDir.c_str());
        return 1;
    }

    if (!player.open(device)) {
        std::fprintf(stderr, "midi_test: failed to open ALSA device '%s'\n", device.c_str());
        return 1;
    }

    const auto names = player.soundNames();

    // ── play each sound once ──────────────────────────────────────────────────
    std::fprintf(stdout, "\n── one-shot playback ───────────────────────────────\n");
    for (const auto& name : names) {
        soundDone.store(false);
        std::fprintf(stdout, "Playing '%s'...\n", name.c_str());
        player.play(name);

        std::unique_lock<std::mutex> lk(doneMtx);
        doneCv.wait_for(lk, std::chrono::seconds(30),
                        [&] { return soundDone.load(); });
    }

    // ── loop the first sound for 3 seconds, then stop ─────────────────────────
    if (!names.empty()) {
        std::fprintf(stdout, "\n── loop test (3 s) ─────────────────────────────────\n");
        soundDone.store(false);
        std::fprintf(stdout, "Looping '%s'...\n", names.front().c_str());
        player.play(names.front(), /*loop=*/true);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::fprintf(stdout, "Stopping...\n");
        player.stop();

        // Wait for STOPPED confirmation.
        std::unique_lock<std::mutex> lk(doneMtx);
        doneCv.wait_for(lk, std::chrono::seconds(5),
                        [&] { return soundDone.load(); });
    }

    player.close();
    dashcam::log::shutdown();
    std::fprintf(stdout, "\nmidi_test: done\n");
    return 0;
}
