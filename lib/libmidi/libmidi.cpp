#include "libmidi.h"

#include <alsa/asoundlib.h>
#include <dirent.h>

#include <algorithm>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>

// ─── file-local helpers ───────────────────────────────────────────────────────

namespace {

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

// Read helpers — little-endian, no UB.
static bool readLE16(std::FILE* f, uint16_t& v) {
    uint8_t b[2];
    if (std::fread(b, 1, 2, f) != 2) return false;
    v = static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
    return true;
}

static bool readLE32(std::FILE* f, uint32_t& v) {
    uint8_t b[4];
    if (std::fread(b, 1, 4, f) != 4) return false;
    v = static_cast<uint32_t>(b[0])
      | (static_cast<uint32_t>(b[1]) << 8)
      | (static_cast<uint32_t>(b[2]) << 16)
      | (static_cast<uint32_t>(b[3]) << 24);
    return true;
}

} // namespace

// ─── dashcam::midi ────────────────────────────────────────────────────────────

namespace dashcam::midi {

// ─── WAV loader ───────────────────────────────────────────────────────────────
//
// Parses RIFF/WAVE chunk-by-chunk so files with extra metadata chunks
// (LIST, fact, etc.) between fmt and data are handled correctly.
// Only PCM (audioFormat=1), 16-bit samples are accepted.

bool MidiPlayer::loadWav(const std::string& path, SoundBuffer& out,
                         const dashcam::log::LogCallback& log) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "loadWav: cannot open '%s': %s", path.c_str(), ::strerror(errno));
        return false;
    }

    // RIFF / WAVE identity
    char id[4]; uint32_t sz;
    if (std::fread(id, 1, 4, f) != 4 || std::memcmp(id, "RIFF", 4) != 0 ||
        !readLE32(f, sz) ||
        std::fread(id, 1, 4, f) != 4 || std::memcmp(id, "WAVE", 4) != 0) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "loadWav: not a RIFF/WAVE file: %s", path.c_str());
        std::fclose(f);
        return false;
    }

    uint16_t audioFormat = 0, numChannels = 0, bitsPerSample = 0;
    uint32_t sampleRate  = 0;
    bool fmtFound = false, dataFound = false;

    while (!dataFound) {
        char     chunkId[4];
        uint32_t chunkSz;
        if (std::fread(chunkId, 1, 4, f) != 4 || !readLE32(f, chunkSz)) break;

        long chunkStart = std::ftell(f);

        if (std::memcmp(chunkId, "fmt ", 4) == 0) {
            uint16_t dummy16; uint32_t dummy32;
            if (!readLE16(f, audioFormat)   ||
                !readLE16(f, numChannels)   ||
                !readLE32(f, sampleRate)    ||
                !readLE32(f, dummy32)       ||  // byteRate
                !readLE16(f, dummy16)       ||  // blockAlign
                !readLE16(f, bitsPerSample)) {
                doLog(log, dashcam::log::LogLevel::ERROR,
                      "loadWav: truncated fmt chunk: %s", path.c_str());
                std::fclose(f); return false;
            }
            if (audioFormat != 1) {
                doLog(log, dashcam::log::LogLevel::ERROR,
                      "loadWav: only PCM (audioFormat=1) supported, got %u: %s",
                      audioFormat, path.c_str());
                std::fclose(f); return false;
            }
            if (bitsPerSample != 16) {
                doLog(log, dashcam::log::LogLevel::ERROR,
                      "loadWav: only 16-bit depth supported, got %u: %s",
                      bitsPerSample, path.c_str());
                std::fclose(f); return false;
            }
            fmtFound = true;

        } else if (std::memcmp(chunkId, "data", 4) == 0 && fmtFound) {
            const std::size_t nSamples = chunkSz / sizeof(int16_t);
            out.samples.resize(nSamples);
            if (std::fread(out.samples.data(), sizeof(int16_t), nSamples, f) != nSamples) {
                doLog(log, dashcam::log::LogLevel::ERROR,
                      "loadWav: truncated data chunk: %s", path.c_str());
                std::fclose(f); return false;
            }
            out.channels   = numChannels;
            out.sampleRate = sampleRate;
            dataFound = true;
            continue;  // skip the seek below
        }

        // Skip to end of this chunk (RIFF pads odd-sized chunks with one byte).
        std::fseek(f, chunkStart + static_cast<long>(chunkSz) + (chunkSz & 1u), SEEK_SET);
    }

    std::fclose(f);

    if (!fmtFound || !dataFound) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "loadWav: missing fmt or data chunk: %s", path.c_str());
        return false;
    }
    return true;
}

// ─── directory scan ───────────────────────────────────────────────────────────

int MidiPlayer::loadSoundsFromDirectory(const std::string& dir) {
    sounds_.clear();
    commonChannels_   = 0;
    commonSampleRate_ = 0;

    DIR* d = ::opendir(dir.c_str());
    if (!d) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "loadSoundsFromDirectory: cannot open '%s': %s",
              dir.c_str(), ::strerror(errno));
        return 0;
    }

    int count = 0;
    struct dirent* ent;
    while ((ent = ::readdir(d)) != nullptr) {
        const char* name = ent->d_name;
        const char* dot  = std::strrchr(name, '.');
        if (!dot || std::strcmp(dot, ".wav") != 0) continue;

        SoundBuffer buf;
        buf.name.assign(name, static_cast<std::size_t>(dot - name));

        if (!loadWav(dir + "/" + name, buf, log_)) continue;

        if (count == 0) {
            commonChannels_   = buf.channels;
            commonSampleRate_ = buf.sampleRate;
        } else if (buf.channels != commonChannels_ || buf.sampleRate != commonSampleRate_) {
            doLog(log_, dashcam::log::LogLevel::WARN,
                  "loadSoundsFromDirectory: '%s' is %u Hz/%u ch but first sound is "
                  "%u Hz/%u ch — skipping (all sounds must share the same format)",
                  name, buf.sampleRate, buf.channels,
                  commonSampleRate_, commonChannels_);
            continue;
        }

        const std::size_t frames = buf.samples.size() / buf.channels;
        doLog(log_, dashcam::log::LogLevel::INFO,
              "  loaded '%-24s'  %u Hz  %u ch  %zu frames  (~%.1f s)",
              buf.name.c_str(), buf.sampleRate, buf.channels,
              frames, static_cast<double>(frames) / static_cast<double>(buf.sampleRate));

        sounds_.push_back(std::move(buf));
        ++count;
    }
    ::closedir(d);

    doLog(log_, dashcam::log::LogLevel::INFO,
          "loadSoundsFromDirectory: %d sound(s) loaded from '%s'", count, dir.c_str());
    return count;
}

// ─── constructor / destructor ─────────────────────────────────────────────────

MidiPlayer::MidiPlayer()  = default;
MidiPlayer::~MidiPlayer() { close(); }

void MidiPlayer::setLogCallback(dashcam::log::LogCallback cb) {
    log_ = std::move(cb);
}

void MidiPlayer::setPlaybackCallback(PlaybackCallback cb) {
    std::lock_guard<std::mutex> lk(mtx_);
    playCb_ = std::move(cb);
}

std::vector<std::string> MidiPlayer::soundNames() const {
    std::vector<std::string> names;
    names.reserve(sounds_.size());
    for (const auto& s : sounds_) names.push_back(s.name);
    return names;
}

// ─── ALSA open / close ────────────────────────────────────────────────────────

bool MidiPlayer::open(const std::string& device) {
    if (pcm_) {
        doLog(log_, dashcam::log::LogLevel::WARN, "MidiPlayer::open: already open");
        return true;
    }
    if (sounds_.empty()) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "MidiPlayer::open: no sounds loaded — call loadSoundsFromDirectory() first");
        return false;
    }

    snd_pcm_t* pcm = nullptr;
    int rc = snd_pcm_open(&pcm, device.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "MidiPlayer::open: snd_pcm_open('%s') failed: %s",
              device.c_str(), snd_strerror(rc));
        return false;
    }

    // snd_pcm_set_params is the simple all-in-one configuration call.
    // Soft resampling (arg 5 = 1) lets "default" work even when the hardware
    // clock doesn't match the WAV sample rate.
    // 50 000 µs (50 ms) latency gives a comfortable buffer without audible lag
    // for alert sounds.
    rc = snd_pcm_set_params(pcm,
                            SND_PCM_FORMAT_S16_LE,
                            SND_PCM_ACCESS_RW_INTERLEAVED,
                            commonChannels_,
                            commonSampleRate_,
                            /*soft_resample=*/1,
                            /*latency_us=*/50000);
    if (rc < 0) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "MidiPlayer::open: snd_pcm_set_params failed: %s", snd_strerror(rc));
        snd_pcm_close(pcm);
        return false;
    }

    pcm_ = pcm;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        running_  = true;
        cmdReady_ = false;
        playing_  = false;
        curName_.clear();
    }
    thread_ = std::thread(&MidiPlayer::playbackThread, this);

    doLog(log_, dashcam::log::LogLevel::INFO,
          "MidiPlayer::open: '%s'  %u Hz  %u ch",
          device.c_str(), commonSampleRate_, commonChannels_);
    return true;
}

void MidiPlayer::close() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!running_) {
            if (pcm_) {
                snd_pcm_close(static_cast<snd_pcm_t*>(pcm_));
                pcm_ = nullptr;
            }
            return;
        }
        running_  = false;
        cmdReady_ = true;  // wake/interrupt the write loop
    }
    cv_.notify_one();
    if (thread_.joinable()) thread_.join();

    if (pcm_) {
        snd_pcm_drop(static_cast<snd_pcm_t*>(pcm_));
        snd_pcm_close(static_cast<snd_pcm_t*>(pcm_));
        pcm_ = nullptr;
    }
    doLog(log_, dashcam::log::LogLevel::INFO, "MidiPlayer::close: done");
}

// ─── play / stop ─────────────────────────────────────────────────────────────

bool MidiPlayer::play(const std::string& soundName, bool loop) {
    if (!pcm_) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "MidiPlayer::play: device not open");
        return false;
    }

    const SoundBuffer* found = nullptr;
    for (const auto& s : sounds_)
        if (s.name == soundName) { found = &s; break; }

    if (!found) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "MidiPlayer::play: unknown sound '%s'", soundName.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        pendingCmd_ = {found, loop};
        cmdReady_   = true;
    }
    cv_.notify_one();
    return true;
}

void MidiPlayer::stop() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!playing_ && !cmdReady_) return;  // already idle, nothing queued
        pendingCmd_ = {nullptr, false};
        cmdReady_   = true;
    }
    cv_.notify_one();
}

bool MidiPlayer::isPlaying() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return playing_;
}

std::string MidiPlayer::currentSound() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return curName_;
}

// ─── callback helper ─────────────────────────────────────────────────────────

void MidiPlayer::fireCallback(const std::string& name, PlaybackEvent ev) {
    PlaybackCallback cb;
    { std::lock_guard<std::mutex> lk(mtx_); cb = playCb_; }
    if (cb) cb(name, ev);
}

// ─── playback thread ─────────────────────────────────────────────────────────
//
// State machine:
//
//   IDLE: wait on cv_ until cmdReady_ || !running_
//
//   On stop-cmd (buf == nullptr):
//     If playing_, drain ALSA, fire STOPPED, mark idle.
//
//   On play-cmd (buf != nullptr):
//     Mark as playing, prepare ALSA, fire STARTED.
//     Write loop (kPeriod frames at a time):
//       Poll cmdReady_ each period — break if interrupted.
//       Recover ALSA underruns via snd_pcm_recover.
//     After the write loop:
//       If interrupted: fire STOPPED, leave cmdReady_ set for next iteration.
//       If error:       fire ERROR, mark idle.
//       If natural end: fire STOPPED, mark idle.
//       If loop and not interrupted: fire LOOPED, restart.

void MidiPlayer::playbackThread() {
    snd_pcm_t* pcm = static_cast<snd_pcm_t*>(pcm_);
    constexpr snd_pcm_uframes_t kPeriod = 1024;  // ~23 ms at 44100 Hz

    while (true) {
        // ── wait for a command ────────────────────────────────────────────────
        Cmd cmd;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return cmdReady_ || !running_; });
            if (!running_) break;
            cmd       = pendingCmd_;
            cmdReady_ = false;
        }

        // ── stop command ──────────────────────────────────────────────────────
        if (!cmd.buf) {
            std::string prev;
            bool wasPlaying;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                wasPlaying = playing_;
                prev       = curName_;
                playing_   = false;
                curName_.clear();
            }
            if (wasPlaying) {
                snd_pcm_drop(pcm);
                snd_pcm_prepare(pcm);
                fireCallback(prev, PlaybackEvent::STOPPED);
            }
            continue;
        }

        // ── play command ──────────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(mtx_);
            playing_ = true;
            curName_ = cmd.buf->name;
        }
        snd_pcm_prepare(pcm);
        fireCallback(cmd.buf->name, PlaybackEvent::STARTED);

        const SoundBuffer&      buf   = *cmd.buf;
        const snd_pcm_uframes_t total = buf.samples.size() / buf.channels;
        bool interrupted = false;
        bool error       = false;

        // ── write loop ────────────────────────────────────────────────────────
        do {
            snd_pcm_uframes_t pos = 0;

            while (pos < total) {
                // Check for a pre-empting command without blocking.
                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    if (cmdReady_) { interrupted = true; break; }
                }

                const snd_pcm_uframes_t toWrite = std::min(total - pos, kPeriod);
                snd_pcm_sframes_t n = snd_pcm_writei(
                    pcm, buf.samples.data() + pos * buf.channels, toWrite);

                if (n < 0) {
                    // snd_pcm_recover handles EPIPE (underrun) and ESTRPIPE (suspend).
                    n = snd_pcm_recover(pcm, static_cast<int>(n), /*silent=*/0);
                    if (n < 0) {
                        doLog(log_, dashcam::log::LogLevel::ERROR,
                              "playbackThread: snd_pcm_writei: %s", snd_strerror(static_cast<int>(n)));
                        error = true;
                        break;
                    }
                    // After recovery snd_pcm_writei must be retried from the same pos.
                } else {
                    pos += static_cast<snd_pcm_uframes_t>(n);
                }
            }

            if (interrupted || error) break;

            if (cmd.loop) fireCallback(buf.name, PlaybackEvent::LOOPED);

        } while (cmd.loop);

        // ── post-playback state ───────────────────────────────────────────────
        if (error) {
            {
                std::lock_guard<std::mutex> lk(mtx_);
                playing_ = false;
                curName_.clear();
            }
            fireCallback(buf.name, PlaybackEvent::ERROR);
        } else if (interrupted) {
            // A new command (play or stop) is already in pendingCmd_.
            // Fire STOPPED for the sound we just abandoned, then let the
            // next outer-loop iteration consume the pending command.
            {
                std::lock_guard<std::mutex> lk(mtx_);
                playing_ = false;
                curName_.clear();
            }
            fireCallback(buf.name, PlaybackEvent::STOPPED);
        } else {
            // Natural end of a non-looping sound.
            {
                std::lock_guard<std::mutex> lk(mtx_);
                playing_ = false;
                curName_.clear();
            }
            fireCallback(buf.name, PlaybackEvent::STOPPED);
        }
    }

    // Thread exit — ensure state is consistent.
    {
        std::lock_guard<std::mutex> lk(mtx_);
        playing_ = false;
        curName_.clear();
    }
}

} // namespace dashcam::midi
