/**
 * @file libmidi.h
 * @brief WAV sample player with MIDI-style note-on / note-off control.
 *
 * MidiPlayer loads PCM WAV files from a directory into memory and plays them
 * on demand through an ALSA PCM device.  At most one sound plays at a time;
 * calling play() while a sound is active stops it first.  Looping, lifecycle
 * callbacks, and graceful shutdown are all supported.
 *
 * Typical usage:
 * @code
 *   dashcam::midi::MidiPlayer player;
 *   player.setLogCallback(dashcam::log::getCallback());
 *   player.setPlaybackCallback([](const std::string& name,
 *                                 dashcam::midi::PlaybackEvent ev) {
 *       if (ev == dashcam::midi::PlaybackEvent::STOPPED)
 *           // recording done, gate is closed, etc.
 *   });
 *
 *   player.loadSoundsFromDirectory("/data/sounds");
 *   player.open();              // "default" ALSA device
 *
 *   player.play("alert");       // one-shot
 *   player.play("engine_idle", true);  // loop until stop()
 *   player.stop();
 *
 *   player.close();
 * @endcode
 *
 * @note Only 16-bit signed PCM WAV files are supported.  All sounds in a
 *       directory must share the same sample rate and channel count; files
 *       that differ from the first loaded sound are skipped with a warning.
 * @note Callback is invoked from the playback thread.  Keep it short and
 *       non-blocking; do not call play() or stop() from inside it.
 * @note Requires ALSA (libasound2).  On Jetson JetPack 6.2 it is always
 *       present; install libasound2-dev for the build headers.
 */

#ifndef LIBMIDI_H
#define LIBMIDI_H

#include "liblog.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dashcam::midi {

// ─── event enum ───────────────────────────────────────────────────────────────

/**
 * @brief Lifecycle events delivered via the playback callback.
 */
enum class PlaybackEvent {
    STARTED,  ///< A sound has begun playing (fired before the first sample).
    STOPPED,  ///< Playback reached its end or was stopped via stop() / play().
    LOOPED,   ///< A looping sound wrapped back to the first sample.
    ERROR,    ///< An unrecoverable ALSA write error occurred; playback aborted.
};

/**
 * @brief Callback signature for playback lifecycle events.
 *
 * @param soundName  Basename (without extension) of the sound involved.
 * @param event      What happened.
 */
using PlaybackCallback = std::function<void(const std::string& soundName,
                                            PlaybackEvent event)>;

// ─── MidiPlayer ───────────────────────────────────────────────────────────────

/**
 * @brief ALSA-backed WAV sample player with note-on / note-off semantics.
 *
 * Thread safety:
 *   - play(), stop(), isPlaying(), currentSound() are safe from any thread.
 *   - loadSoundsFromDirectory() and open() / close() must be called from a
 *     single owner thread (not re-entrant).
 *   - The PlaybackCallback is invoked from the internal playback thread.
 */
class MidiPlayer {
public:
    MidiPlayer();
    ~MidiPlayer();

    MidiPlayer(const MidiPlayer&)            = delete;
    MidiPlayer& operator=(const MidiPlayer&) = delete;

    // ─── configuration ────────────────────────────────────────────────────────

    /** @brief Inject a logger.  No-op if not set. */
    void setLogCallback(dashcam::log::LogCallback cb);

    /**
     * @brief Register the playback event callback.
     *
     * Must be set before open() to guarantee no events are missed.
     * Replaces any previously registered callback.
     */
    void setPlaybackCallback(PlaybackCallback cb);

    // ─── sound library ────────────────────────────────────────────────────────

    /**
     * @brief Scan @p dir for *.wav files and load them into memory.
     *
     * The sound name used with play() is the filename basename without the
     * ".wav" extension (e.g. "alert.wav" → "alert").
     *
     * Must be called before open().  Safe to call multiple times — each call
     * replaces the previously loaded library.
     *
     * @param dir  Directory path (absolute or relative to cwd).
     * @return     Number of sounds successfully loaded; 0 on error or if the
     *             directory contains no suitable .wav files.
     */
    int loadSoundsFromDirectory(const std::string& dir);

    /** @brief Names of all loaded sounds, in load order. */
    std::vector<std::string> soundNames() const;

    // ─── device lifecycle ─────────────────────────────────────────────────────

    /**
     * @brief Open the ALSA PCM device and start the playback thread.
     *
     * Must be called after loadSoundsFromDirectory() and before play().
     * The ALSA device is configured to match the loaded sounds' sample rate
     * and channel count; soft resampling is enabled so "default" works even
     * if the hardware runs at a different rate.
     *
     * @param device  ALSA PCM device name.  "default" works in most setups;
     *                use "plughw:0,0" to bypass the dmix/pulse layer.
     * @return true on success; false if no sounds are loaded or ALSA init fails.
     */
    bool open(const std::string& device = "default");

    /**
     * @brief Stop any active playback, join the playback thread, and close
     *        the ALSA device.
     *
     * Blocks until the playback thread exits.  All in-flight callbacks
     * complete before this function returns.  Safe to call even if open()
     * was never called or already failed.
     */
    void close();

    // ─── playback control ─────────────────────────────────────────────────────

    /**
     * @brief Start playing a sound (note-on).
     *
     * Non-blocking — control returns immediately while the playback thread
     * queues the audio.  If a sound is already playing, it is stopped first
     * (STOPPED callback fires before STARTED for the new sound).
     *
     * @param soundName  Name as returned by soundNames().
     * @param loop       If true, the sound repeats until stop() is called.
     * @return false if the name is not found or the device is not open.
     */
    bool play(const std::string& soundName, bool loop = false);

    /**
     * @brief Stop the current sound (note-off).
     *
     * Non-blocking.  STOPPED callback fires from the playback thread shortly
     * after this returns.  No-op if nothing is playing.
     */
    void stop();

    // ─── query ────────────────────────────────────────────────────────────────

    /** @brief true if a sound is currently playing. */
    bool        isPlaying()    const;

    /** @brief Basename of the sound currently playing; empty if idle. */
    std::string currentSound() const;

private:
    // ─── sound buffer ────────────────────────────────────────────────────────

    struct SoundBuffer {
        std::string          name;
        std::vector<int16_t> samples;    ///< Interleaved PCM S16_LE
        uint32_t             channels;
        uint32_t             sampleRate;
    };

    // ─── playback command ────────────────────────────────────────────────────

    struct Cmd {
        const SoundBuffer* buf  = nullptr;  ///< nullptr = stop
        bool               loop = false;
    };

    // ─── members ─────────────────────────────────────────────────────────────

    dashcam::log::LogCallback log_{};
    PlaybackCallback          playCb_{};

    std::vector<SoundBuffer>  sounds_;
    uint32_t                  commonChannels_   = 0;
    uint32_t                  commonSampleRate_ = 0;

    void* pcm_ = nullptr;  ///< snd_pcm_t* — owned; null when closed

    mutable std::mutex      mtx_;
    std::condition_variable cv_;

    Cmd         pendingCmd_;
    bool        cmdReady_ = false;  ///< true when pendingCmd_ has been updated
    bool        running_  = false;  ///< set to false to terminate the thread
    bool        playing_  = false;  ///< updated by playback thread only
    std::string curName_;           ///< updated by playback thread only

    std::thread thread_;

    // ─── private helpers ─────────────────────────────────────────────────────

    void playbackThread();
    void fireCallback(const std::string& name, PlaybackEvent ev);

    static bool loadWav(const std::string& path, SoundBuffer& out,
                        const dashcam::log::LogCallback& log);
};

} // namespace dashcam::midi

#endif // LIBMIDI_H
