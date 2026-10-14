# ES3C28P microphone → Jetson voice commands

Status: implementation plan; updated 2026-10-14 after tap-to-speak confirmation. No voice firmware, recognizer, models,
or runtime configuration are installed by this document. The existing video
plan and frozen video protocol remain unchanged.

## Confirmed scope

- Version one recognizes **English** commands that select the five existing
  screen tabs. The Jetson recognizes speech and resolves a typed navigation
  intent; the ES3C28P applies that intent and reports the actual selected tab.
- Audio capture/recognition and video preview need not run concurrently. Use
  exclusive voice and preview modes initially. Camera recording continues.
- Activation is confirmed as **tap-to-speak**: tap the microphone, wait for
  “Listening”, speak one English command, and return to the selected tab.
  End capture automatically after speech followed by silence, with explicit
  Done and Cancel controls. The microphone is inactive between interactions.
- No activation, language or initial-command-scope clarification remains open.
  Timing, gain and model/backend choices below are engineering defaults to
  validate on the actual microphone and Jetson.
- Proposed deployment: offline recognition on Jetson, no cloud service. Audio
  stays in bounded RAM and is discarded after the interaction. Diagnostic WAV
  capture is an explicit development option, disabled by default.

## Repository and hardware findings

The [manufacturer's pin table](https://www.lcdwiki.com/2.8inch_ESP32-S3_Display)
and [schematic](https://www.lcdwiki.com/res/ES3C28P/2.8inch_ESP32-S3_Display_Schematic.pdf)
identify the onboard microphone/ES8311 path and I2S input on GPIO6. Reuse MCLK4,
BCLK5 and WS7; GPIO8 is playback data, GPIO1 controls the amplifier, and I2C
SDA16/SCL15 addresses the codec at 0x18. Confirm microphone signal level and
active I2S slot on the physical board before selecting gain or mono extraction.

Current `es3Audio.cpp` allocates TX only, sets DIN unused, and disables ADC
serial output in its codec setup. Capture requires an ADC configuration and RX
channel; toggling a pin or reading the ESP32's analog ADC is insufficient.
Use [Espressif's ES8311 driver](https://github.com/espressif/esp-adf/blob/master/components/esp_codec_dev/device/es8311/es8311.c)
as the register-sequence reference, pinning the reviewed revision during
implementation. Keep the analog microphone path, sample format and clock
coefficients consistent. Do not guess microphone gain or ADC slot placement.

The [ESP-IDF 5.4.1 I2S API](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/api-reference/peripherals/i2s.html)
provides RX DMA. TX and RX on a shared full-duplex peripheral require matching
clock/slot setup; version one instead switches exclusive playback and capture
ownership, which also avoids recording its own speaker output.

Other relevant current behavior:

- `Es3Graphics` has touch navigation but no public remote tab-selection method.
  Existing USB PAGE updates text; it does **not** select the active tab.
- Re-tapping the active SET tab starts calibration. Voice “settings” must only
  select SET; it must never simulate this second-tap calibration gesture.
- `Es3UsbLink` owns USB on core 0 and queues graphics/audio commands to the
  Arduino loop. Preserve that ownership boundary.
- `lib/libes3display` now provides the Jetson's sole tty owner, handshake,
  heartbeat, video credits and callbacks. Extend this owner rather than opening
  the same tty from a speech process or Python helper.
- Preview production consults transport `demand()`. Add an explicit media-mode
  gate there; a tab being LIVE must not accidentally restart preview mid-command.
- The checked host reports L4T R36.5.2. Build the recognizer inside the actual
  project development/runtime container and check its CUDA/compiler versions;
  do not copy older version assumptions from the local skill defaults.

## Version-one navigation vocabulary

Use a small, explicit alias table after English case/punctuation/whitespace
normalization. Match the **whole utterance**, not a substring or nearest phrase.

| Intent | Existing tab index | Accepted examples |
| --- | ---: | --- |
| SelectLive | 0 (LIVE) | “live”, “show live”, “open live”, “go to live” |
| SelectTrip | 1 (TRIP) | “trip”, “trip summary”, “show trip”, “open trip” |
| SelectCameras | 2 (CAM) | “camera”, “cameras”, “show cameras”, “open cameras” |
| SelectSystem | 3 (SYS) | “system”, “show system”, “open system” |
| SelectSettings | 4 (SET) | “settings”, “show settings”, “open settings” |

These are proposed initial aliases, testable independently of ASR. Add aliases
only with speech fixtures and negative examples. Reject multiple commands,
negation (“do not open settings”), conversational questions, unrelated speech,
noise and empty recognition. No arbitrary shell commands, general assistant,
recording start/stop, deletion, camera switching or calibration in version one.
Selecting the current tab is an idempotent success and has no extra side effect.

## End-to-end interaction

1. A microphone affordance is available on every dashboard tab, with visible
   disabled/unavailable state until the host advertises voice readiness. Put it
   in the header/black bar where available; for full-panel video, reveal it with
   navigation controls. Hide it during calibration and its verification screen.
2. On activation, the board creates a request ID and displays “Preparing voice”.
   It immediately blocks local video presentation and asks the Jetson to enter
   voice mode. The host closes preview demand, releases pending JPEG leases,
   completes or aborts the current wire record/frame, and STOP_VIDEOs its token.
   Never cut a partial serial record to insert a new one.
3. The board releases video work through the existing lease-safe paths. Do not
   free a JPEG/image buffer still owned by decode/TFT. The transition is ready
   only after owners report quiescence; avoid blocking the USB or UI thread.
4. Host arms one capture only after it has a free utterance buffer and a ready
   recognizer. The loop stops playback, mutes the amplifier, switches codec/I2S
   to capture, discards the measured ADC settling interval, then signals READY.
   Only then show “Listening”. Proposed preparation deadline: two seconds.
5. Capture 16 kHz signed 16-bit mono PCM. Stream bounded blocks to the existing
   host USB worker. The host performs voice activity detection; for tap mode,
   propose a three-second no-speech timeout, 600 ms trailing silence, and a
   six-second maximum recording interval. The board enforces its own hard cap.
   A visible Done button may end early; Cancel always aborts without navigation.
6. On end-of-speech, the host requests capture stop. The board stops RX, drains
   the already-captured valid blocks, and sends END with exact sample count and
   whole-utterance checksum. A gap, overflow or checksum error invalidates the
   utterance; never silently stitch missing PCM into a command.
7. Jetson recognizes the complete utterance on its worker and maps it to one
   of the five intents. No-speech/unknown/ambiguous input produces a visible
   “Not understood” result without changing the tab.
8. On a valid intent, send a separate SELECT_TAB transaction tagged with the
   interaction ID. The application loop validates that the interaction is still
   active, applies the tab selection, and ACKs the actual index. Distinguish
   touch-origin selection from voice-origin selection so the latter does not
   cancel itself. Report success only after that ACK. Manual navigation during the interaction cancels it;
   a late recognition result cannot override the user's newer touch selection.
9. Release capture resources, restore playback readiness and close voice mode.
   Resume video with fresh negotiation only if the resulting tab is LIVE and
   video was enabled. Other tabs stay in normal hidden-preview state. Failure
   or cancellation before selection keeps the original tab unless the user
   changed it manually; an already-applied selection follows the outcome rules
   in the transition section below.

USB loss/HELLO/reboot invalidates the interaction, queued samples and pending
recognition result. Return to usable local UI; never replay speech or a previous
navigation command on reconnect. An ACK lost after selection can be reconciled
with STATUS; it must not cause the SET calibration action.

## Tap-to-speak UI and timing contract

These details are implementation decisions following the confirmed tap gesture;
thresholds remain tunable during the hardware evaluation.

### Touch and feedback

- Reserve a microphone button on each dashboard page. For the existing 320×180
  preview, use the upper 30-pixel black bar; a proposed hit rectangle is
  x=280..315, y=3..26. Reserve the corresponding header space on text pages.
  Keep its hit area separate from the bottom tab strip and brightness controls.
- When video occupies the whole panel, the first tap reveals navigation and a
  microphone affordance. A separate tap on that affordance starts voice; the
  reveal gesture itself must not start a recording.
- Trigger once on a calibrated touch release inside the same microphone hit
  rectangle where the press started. Ignore drags out, held-touch repeats and
  double-tap bounce. Consume that gesture so it cannot also select a tab.
- While idle, disabled states distinguish “Jetson offline”, “Voice unavailable”
  and “Voice busy”. Do not accept an activation and then wait indefinitely for
  a model to load. Only advertise readiness after both peers are ready.
- Replace the content area with a voice panel while preserving the tab strip.
  Preparing shows “Preparing voice”; Listening shows “Listening” and a simple
  level meter; Recognizing shows “Recognizing”; Applying shows “Opening SYS”
  (or the canonical target tab). UI refresh is bounded, proposed at 10 Hz.
- During Listening, show two distinct buttons: **Done** and **Cancel**, proposed
  rectangles x=12..151 and x=168..307, y=174..213. Disable Done outside Listening;
  keep Cancel active through Preparing, Listening, Draining, Recognizing and
  Applying.
  Repeated microphone taps do not toggle capture or enqueue another interaction.
- After a confirmed selection, show a brief “Opened SYS” result for at most
  750 ms without blocking input. Return to the selected page automatically.
  Keep a concise failure notice on the current page when speech is not understood.
- A tab touch cancels the active interaction, then applies the touched tab.
  Preserve the existing explicit SET re-tap calibration behavior only after
  voice teardown has released capture resources. Voice “settings” never invokes
  that gesture. Calibration and its verification screen do not offer a microphone.

### Audio endpoint rules

The clock for capture/no-speech starts at **READY**, after ADC settling, not
at the initial tap. Show no Listening indicator and accept no speech until
READY has been generated. Use sample indices for PCM duration and wrap-safe
monotonic clocks for transport/UI deadlines.

1. Begin sending all valid PCM blocks from READY; retain the beginning of the
   utterance so VAD startup does not clip the first consonant. There is no
   background microphone recording or pre-roll from before activation.
2. The host analyzes ordered 20 ms blocks. Mark speech after a proposed total
   of 200 ms of VAD-positive audio; validate that this still admits short “live”
   and “trip” utterances. Choose and pin a VAD implementation during the speech
   fixture evaluation, with tests independent of ASR.
3. If speech has not started within 3,000 ms, finish with NoSpeech and skip
   recognition. Once speech starts, 600 ms of trailing silence requests FINISH.
   Noise alone must not force a command; the recognizer and full-phrase mapper
   remain additional rejection gates.
4. Done requests the same bounded FINISH/drain path, even before silence.
   It does not bypass minimum valid speech, PCM integrity or intent validation.
5. At the six-second firmware capture cap, stop sampling with TooLong. Treat
   that interaction as incomplete and do not execute a possibly truncated
   command. Ask for a shorter command on the next activation.
6. Cancel invalidates the interaction immediately and skips ASR/navigation.
   Unsatisfied queued audio grants or late model completion cannot reactivate it.

### Transition table and cleanup

| State | Owner action / visible state | Successful exit | Failure / timeout |
| --- | --- | --- | --- |
| Idle | Board UI ready; host model loaded; microphone off | One tap creates an interaction generation | Disabled when disconnected/unavailable/busy |
| Preparing | Gate preview, quiesce video owners, mute playback, arm RX | READY → Listening | 2 s: cancel and restore |
| Listening | Bounded PCM streaming and level display | Silence or Done → Draining | No speech at 3 s; capture cap at 6 s; overflow/gap/Cancel → restore without action |
| Draining | Stop RX, send remaining complete blocks and END | Verified END → Recognizing | 1 s: discard the utterance and restore |
| Recognizing | Worker transcribes; mapper accepts at most one intent | Match → Applying; NoMatch → restore with notice | 5 s: cancel/discard late output |
| Applying | Send SELECT_TAB with token/generation/index | Applied ACK → Restoring | 1 s: outcome unresolved; reconcile actual tab with STATUS, do not replay |
| Restoring | RX stopped/acknowledged; playback ready; clear mode gate | Idle; restart preview only for LIVE | 1 s: mark voice unavailable, preserve usable UI/controls |

A separate 20 s interaction watchdog bounds the whole transaction, including
repeated state messages. It never extends the six-second sampling cap. Duplicate
messages must not reset phase deadlines. On a phase failure, use one cleanup
path; it is safe to invoke more than once and after partial initialization.
Do not delete/reuse a capture handle or block still leased by another task.
If its owner fails to acknowledge stop, leave that resource quarantined and
voice unavailable rather than freeing it underneath the owner.

The board loop linearizes Cancel/touch and SELECT_TAB. If cancellation is
processed first, the old generation cannot select a page. If selection was
already applied, cancellation cannot undo history: preserve the applied page
unless a later explicit tab touch selects another. Log Applied/Cancelled/
OutcomeUnknown accurately; an ACK timeout alone is not evidence that selection
failed. Keep a small completed-interaction result cache for duplicate requests
within the same voice token, without applying a navigation action twice.

Host cancellation invalidates pending inference output immediately. If the ASR
backend cannot stop promptly, the UI can recover but voice stays busy until
the worker is idle or its supervised process has restarted. It must not queue
unbounded new captures behind a stalled model.

## Board implementation

### Components and APIs

| File/component | Planned change |
| --- | --- |
| `lib/es3Board/es3Board.h` | Add `audioIn = 6` |
| `lib/es3Audio/` | Shared codec/I2S ownership and reversible playback↔capture transitions |
| new `lib/es3Voice/` | Capture worker, fixed PCM pool, interaction state and deadlines |
| `lib/es3UsbLink/` | Voice negotiation, audio framing, upload flow control and cancellation |
| `lib/es3Graphics/` | Microphone UI, listening/result states, explicit `selectPage(index)` |
| `src/main.cpp` | Own mode coordinator; dispatch transitions, timeout and result updates |
| new `bench/mic_capture/` | Microphone gain/slot/clock and end-to-end PCM diagnostic bench |

Proposed ownership contracts:

```cpp
// Application loop: only owner of codec I2C and UI/peripheral transitions.
bool requestCapture(const CaptureConfig& config, uint32_t interactionId);
void requestCaptureStop(StopReason reason);
bool selectPage(uint8_t index); // no touch-gesture side effects
void cancelVoice(CancelReason reason);

// USB task: leases immutable blocks; never calls Wire, SD or TFT.
bool acquireAudioBlock(AudioBlockLease& block);
void releaseAudioBlock(AudioBlockLease& block);

// Capture worker: owns bounded RX reads only while capture handle is leased.
// It acknowledges stop before the loop disables/deletes/reconfigures I2S.
```

Make one coordinator own `Idle/Preparing/Listening/Draining/Recognizing/Applying/
Restoring` states and a monotonically increasing interaction generation. USB,
capture and UI publish events through bounded queues. No task can close an I2S
handle that another task is reading. All codec I2C remains in the loop because
it shares Wire with touch. Capture worker runs on core 0 while JPEG work is
paused; USB and touch remain responsive. While the voice overlay owns the
screen, bypass video stale/placeholder rendering and ordinary page redraws;
neither a stopped video session nor a PAGE text update may erase “Listening”.

Share codec setup between playback and capture instead of creating two owners
of I2S0. During capture, reject new alert playback with Busy (or cancel capture
first for a future explicitly prioritized alert policy). Do not queue/replay old
alerts after recognition. Default feedback is visual, avoiding acoustic echo.

### Audio bounds

- Wire PCM: 16,000 samples/s × 2 bytes = **32,000 bytes/s**.
- One 20 ms block: 320 mono samples, **640 bytes**. I2S may receive stereo slots;
  select the verified ADC slot before publication, with signed sample handling.
- Start with 16 fixed blocks: **10,240 bytes**, approximately 320 ms of PCM.
  DMA buffers must use memory supported by the driver; PSRAM storage is not
  assumed DMA-compatible. No allocation in the sampling hot path.
- On bounded-pool exhaustion or sustained USB backpressure, abort capture and
  report the cause rather than losing audio silently. Read/stop waits have
  finite timeouts; counters expose overruns and malformed/truncated captures.
- Maximum utterance: six seconds = 96,000 samples = **192,000 bytes** on Jetson.
  Board enforces the cap even when host VAD/stop messages disappear.
- Keep video pools allocated initially. Mode exclusion is about ownership and
  processing; freeing/reallocating PSRAM every interaction adds failure cases.

Measure gain, DC offset, clipping and intelligibility at the intended microphone
distance with fan/road noise. Use a fixed conservative gain first; add high-pass
filtering/AGC/noise suppression only when measurements justify it. Discard the
startup transient before READY rather than making the user speak into it.

## Jetson implementation

Add `lib/libvoicecommand/` with three independently testable layers:

1. A bounded PCM assembler validates token/interaction/sample offsets/CRC and
   feeds VAD. One interaction and one recognition job are allowed at a time.
2. An ASR worker holds a loaded model; it never runs inside the USB worker,
   recorder callback or supervisor heartbeat. Completion contains generation,
   transcript, timing and recognizer diagnostics; cancellation makes old output
   ineligible to issue a command even if inference finishes later.
3. An intent mapper converts a normalized complete phrase to `SelectTab{index}`
   or NoMatch. A small dispatcher requests SELECT_TAB via `libes3display` and
   awaits the board's applied result. No shell/string-command execution.

Suggested API shape:

```cpp
bool start(const VoiceCommandConfig&, Es3DisplayLink&);
bool beginInteraction(uint32_t generation);
SubmitResult submitPcm(const PcmBlockView&); // bounded copy, never waits for ASR
void finishInteraction(const AudioEnd&);
void cancelInteraction(uint32_t generation);
VoiceStats stats() const;
void stop(); // cancel, join workers, then free pools/model
```

Extend the single `libes3display` worker to negotiate voice independently of
video support. Voice must remain available when preview is disabled or video
PSRAM allocation failed. Move display-link lifetime outside a preview-only
startup condition in the app integration. `demand()` returns false throughout
voice mode even if the last video VISIBILITY said active. Keep heartbeats and
status working while recording, voice or preview is idle.

Proposed first recognizer: pinned **whisper.cpp**, English-only `tiny.en`, with
`base.en` as the measured accuracy fallback. Its upstream C++ implementation
supports [CUDA builds and quantized models](https://github.com/ggml-org/whisper.cpp).
Benchmark CPU and CUDA on this Jetson/container with recording active; choose
one backend from measured accuracy, p95 latency and resource headroom. Do not
assume a generic upstream CUDA image supports this aarch64 Jetson deployment.
Pin code revision, model filename/hash and license provenance after evaluation;
load/download models during setup, never in a recording/USB callback.

Use explicit English transcription, no translation, bounded decoder work and
no cross-interaction prompt/history. VAD must reject silence before ASR. Treat
recognizer confidence/no-speech metrics as empirical filters, not calibrated
probabilities. A constrained grammar must not force every noisy clip into one
of five valid tabs. Test rejection as carefully as correct command recognition.
If recognition can stall beyond its budget, expose cancellation/abort support
or isolate the model worker in a supervised process with bounded local IPC;
that process still does not own the tty. Limit CPU workers and GPU memory so
recording and existing inference retain headroom in unified RAM.

## USB extension (draft, not a frozen wire contract)

Keep E3 28 controls and E3 29 video bytes unchanged. Do not enlarge existing
CAPABILITIES, alter frozen video reason values, or add a new video demand enum.
Add a separately negotiated voice capability and voice framing, provisionally
**E3 2A**. Allocate numeric command IDs only after checking the current registry
on both sides, and freeze them with shared C++/Python vectors before coding.

Proposed envelope follows the proven bounded layout:

```text
E3 2A | voice_version:u8 | type:u8 | payload_bytes:u16 |
voice_session_token:u32 | interaction_id:u32 | payload | crc16:u16
```

CRC16 uses the existing CCITT-FALSE convention. VOICE_CAPABILITIES is an
E3 28 request/reply advertising PCM format, hard sample/block limits and
supported UI intents. VOICE_ENABLE establishes a fresh nonzero voice-session
token after the host recognizer is ready. Reset token/interaction state on
HELLO/disconnect. Each reconnect negotiates a new token independent of video.

| Message/command | Direction and purpose |
| --- | --- |
| VOICE_CAPABILITIES | Host control query; discover voice independently of video |
| VOICE_ENABLE | Host control; establish voice token only when recognizer ready |
| REQUEST | Board→host; microphone activation and new interaction ID |
| ARM / READY | Host control then board event; synchronize buffer readiness and actual sampling |
| AUDIO | Board→host; first sample index u32, count u16, up to 640 bytes PCM |
| AUDIO_CREDIT | Host→board; cumulative allowed block count, repeats idempotent |
| FINISH / END | Host stop request; board final sample count, CRC32 and stop reason |
| CANCEL | Either direction; invalidate one interaction and release its resources |
| SELECT_TAB | Host control; token/interaction/index; ACK only after application-loop selection |
| RESULT / DONE | Host result display then board cleanup acknowledgement |

Give the host assembler initial grants for a small fixed window (proposed eight
blocks). Replenish only after accepting data into its bounded utterance buffer;
cap total grants at 300 blocks for six seconds. The board retains unsent samples
in its fixed pool. ACK/grant stalls cannot block capture indefinitely: pool
exhaustion cancels. END follows all sent AUDIO and carries a count/checksum
covering exactly that sequence. An incomplete/gapped utterance never reaches
intent dispatch. Control/heartbeat/cancel traffic takes priority between audio
records; partial writes complete before another record starts. AUDIO needs 662 bytes
including this header and CRC (4-byte offset + 2-byte count + 640-byte PCM);
the current 136-byte firmware control TX buffer is insufficient. Add a bounded
voice-capable TX staging buffer without changing the frozen control maximum.
Bound every other voice payload explicitly before freezing the protocol.

Extend a top-level mux to recognize controls, video and voice; do not pass a
new voice record through the old parser's garbage recovery, which could scan
PCM for embedded control/video magic. Supported length-checked payloads are
opaque until CRC validation. Update host/firmware copies together, leaving
existing golden vectors unchanged and adding new voice vectors. Test all three
magic sequences embedded in audio and in partially written/interleaved traffic.

Older peers that reject the voice query stay usable for video/controls and
show voice unavailable. Runtime mode failures do not restart the recorder.

## Configuration and defaults to validate

Propose a `<VoiceCommands>` config separate from `<DisplayPreview>`:

| Field | Proposed default |
| --- | --- |
| Enabled | false until board capture and recognizer are validated |
| Language / Model | en / pinned tiny.en; base.en only if measured necessary |
| Backend / Workers | benchmark-selected CPU or CUDA / bounded worker count |
| Activation | tap; confirmed user preference |
| SampleRate / BlockMs | 16000 / 20; fixed mono S16LE wire format |
| MaxUtteranceMs | 6000, firmware hard cap |
| NoSpeechTimeoutMs / EndSilenceMs | 3000 / 600, tune with fixtures |
| PrepareTimeoutMs / RecognitionTimeoutMs | 2000 / 5000, provisional deadlines |
| MinimumSpeechMs | 200 of VAD-positive audio, tune for short words such as “live” |
| DrainTimeoutMs / ApplyTimeoutMs / RestoreTimeoutMs | 1000 / 1000 / 1000 |
| InteractionTimeoutMs | 20000 wall-clock maximum, separate from the six-second sample cap |
| ResultDisplayMs | 750; no blocking delay and touch remains active |
| CaptureDiagnostics | false; no default audio/transcript retention |
| AllowedIntents | only the five tab selectors |

Changing ASR language/model does not silently expand allowed actions. Keep
setup paths/config validation explicit and reject missing models before
advertising readiness. Default compilation must support disabling the voice
library without breaking recording or the existing board firmware.

## Implementation order and reviewable deliverables

Each milestone should leave a buildable increment with tests and a short
implementation note; keep this document as the design rather than an assertion
that those steps have already been completed.

| Step | ES3C28P deliverable | Jetson deliverable | Completion evidence |
| --- | --- | --- | --- |
| 1. Select tabs explicitly | `selectPage(index)` and origin-aware navigation; no SET calibration side effect | Typed tab-selector API and fake-board test | All five indices, same-tab idempotency, invalid index and busy/calibrating cases |
| 2. Capture audio independently | Shared codec owner, RX worker and microphone bench | Development PCM/WAV diagnostic receiver using the existing tty owner | Correct 16 kHz signed samples, gain/slot verified, bounded stop, playback restored |
| 3. Freeze voice transport | Portable voice types and top-level three-protocol parser | Matching codec, one-utterance assembler and block grants | Shared golden vectors plus negative framing, capacity and stale-token tests |
| 4. Add tap-mode coordination | Button/panel, mode states, sample cap and cancellation | Preview gating, voice handshake and interaction lifecycle | Automated state-machine tests and hardware mode transitions without ASR |
| 5. Recognize English | Show typed progress/result; no onboard ASR required | Pinned ASR/VAD, exact-alias intent mapper and model setup | Positive/negative fixture results, measured latency and resource use |
| 6. Connect and soak | Apply SELECT_TAB only for a live interaction and send applied ACK | Dispatch accepted intent; reconcile ACK loss; resume demand | Mixed voice/preview/recording acceptance run and recovery tests |

Host file changes should be scoped to `lib/libvoicecommand/`, the existing
`lib/libes3display/` transport, `lib/libconfig/`, `src/dashcam_v0_4.cpp`, test
fixtures/targets and build/dependency packaging. Voice mode must not call
`recorder.stop()` or alter the camera pipeline. Board files are listed in the
component table above; extend Linux/Windows/PlatformIO builds only for actual
new dependencies. Store model assets in a configured model directory, with
reproducible checksums and installation instructions rather than committing
large model binaries to this board directory.

Additional tap-specific acceptance cases:

- One normal tap starts one interaction; a hold, drag-out, bounce or rapid
  double-tap does not start a second capture.
- Full-panel video reveal followed by a microphone tap works without selecting
  an underlying tab. The ordinary letterboxed preview button stays in its bar.
- Silence exits without ASR; Done before valid speech cannot select a tab;
  short valid words remain recognizable; six-second overrun selects nothing.
- Done/Cancel/automatic endpoint arriving together yield one terminal result.
  A manual tab touch racing with an ASR result leaves the user's latest explicit
  navigation in control, using the loop ordering described above.
- Cancellation and USB loss at every phase release or quarantine all resources;
  new sessions reject old audio, credits, SELECT_TAB requests and results.
- A recognition stall keeps heartbeats, touch and recording healthy. A restored
  UI does not imply that a still-busy recognizer can accept another utterance.
- Every mode exit stops microphone acquisition. Listening never continues
  invisibly after leaving the voice panel, disconnecting or entering calibration.

## Delivery milestones and acceptance

1. **Mic bring-up:** add capture bench; validate GPIO6/clock/slot/sign/gain and
   obtain six-second WAVs from real board audio. Measure sample count, clipping,
   dropped samples and return to speaker playback. Test absent/failed codec.
2. **Transport/mode tests:** freeze new protocol; simulate partial/coalesced
   reads/writes, magic inside PCM, CRC/gaps/duplicates/overflow, old tokens,
   handshake timeouts, cancel at every state and disconnect while draining.
   Confirm preview buffers are released safely and voice does not need video.
3. **Recognizer evaluation:** collect English commands from intended speakers
   at real distance plus fan/road/radio/irrelevant speech. Compare tiny.en and
   base.en with recording enabled. Record per-tab confusion, false acceptance,
   rejection rate, latency, memory and CPU/GPU/thermal use; then pin model/backend.
4. **Navigation integration:** explicit selection API + SELECT_TAB + applied
   ACK. Verify all five tabs, repeated current-tab commands, SET without
   calibration, manual-touch cancellation, late results and lost ACK recovery.
5. **End-to-end resilience:** 100 consecutive interactions and a 30-minute
   mixed voice/preview/recording run. Include all tabs, calibration, USB loss,
   host restart, missing model, ASR failure, speaker requests and camera recovery.
   No memory growth, stuck mode, replayed command or recording restart caused
   by voice. Resume video on LIVE and leave it hidden on other selected tabs.

Proposed release targets (not measured): at least 95% correct tab selections
on an agreed positive test set; zero tab changes across a defined negative set
of at least 100 silence/noise/irrelevant/negated clips; p95 end-of-speech to
applied-ACK below two seconds for the chosen backend. Report dataset size and
conditions—zero failures in a finite set is not a guarantee of zero false
activations. Touch/cancel should respond within 100 ms, and the recorder must
match its preview-off health baseline with no voice-induced restarts.

Record implementation/measurements in a separate voice implementation note.
Do not mark milestones complete based solely on this design document.
