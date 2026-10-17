# Jetson → ES3C28P video streaming review, 2026-10-17

Scope: recorder MJPEG tap (`librecord/libsegment.cpp`), preview submission,
conversion and lease pools (`libvideopreview`), USB transport (`libes3display`),
and v0.4 startup/recovery/shutdown integration. The pending navigation/panning
work and previous ES3C28P firmware work were preserved. No design plan or wire
header changed. No deployment, camera capture or firmware flash was performed.

## Findings, in severity order

1. **HIGH — wake descriptor could close underneath a producer.**
   `Es3DisplayLink::offerFrame()` and the other producers publish under the
   handoff mutex, then call `wake()`. `stop()` could join the worker and close
   the eventfd between those operations or between the descriptor check and
   write. Besides a C++ data race, descriptor reuse could send eight bytes to
   an unrelated file. A dedicated wake mutex now serialises descriptor
   publication, nonblocking writes and close. Worker descriptor reads remain
   protected by launch/join ordering. No handoff lock is held during wake.
   The regression pauses a producer immediately before its eventfd syscall,
   runs stop concurrently and checks that the descriptor remains open until
   the syscall completes. Opening the descriptor now follows allocating setup
   so an allocation exception cannot leak it before the worker exists.

2. **MEDIUM — TX fairness budget introduced artificial sleeps.**
   `pumpTx()` does at most 64 build/write steps per pass, correctly giving RX
   and timers a turn. It previously then polled for up to 100 ms even when
   more bytes were immediately writable. A sufficiently large JPEG with small
   negotiated chunks could exceed the assembly deadline on a fast link.
   Budget exhaustion now requests one zero-timeout poll, then resumes work;
   EAGAIN and kernel-queue pressure retain their normal waits. The regression
   sends one >30 KB image in 64-byte chunks, with an 8 KiB kernel-queue limit
   to exercise the work budget independently of queue throttling, and a 500 ms
   assembly timeout. No recurring producer wakeup masks the defect.

3. **MEDIUM — map credit starvation escaped recovery.**
   `outstandingWork()` checked only the camera pending lease once all started
   frames had received RESULT. A map image waiting behind a lost CREDIT could
   therefore remain stuck indefinitely. It now checks the active surface's
   queue, including panning background/chrome. Session-start suppression also
   covers START_MAP and START_PAN. The regression withholds credits after a
   completed whole-map image and verifies renegotiation with another map
   waiting. Panning shares that recovery predicate and remains covered by
   the full integration suite.

4. **MEDIUM — duplicate RESULT Ok became a false drop.**
   The RESULT handler treated any second RESULT as a terminal failure. A replay
   of Ok removed the outstanding frame, so the real PRESENTED became unknown.
   Repeated Ok now leaves the frame waiting, without incrementing completed
   counts or refreshing progress. A later non-Ok RESULT still terminates a
   decode/visibility failure. `resultReasons` remains a count of received
   records, including duplicates. The regression injects duplicate Ok records
   before PRESENTED and verifies one completion and no drop.

Expired assembly deadlines are also excluded from polling once ABORT is pending;
credit deadlines are excluded while renegotiating/stopping. This prevents already
handled deadlines from forcing repeated zero-timeout polls while the tty is blocked.

## Refactor and ownership

`buildVideo()` now handles eligibility and pose interleaving, `nextImage()` selects
under the handoff lock, `startImage()` acquires a compatible lease and spends a
credit only after START encoding succeeds, and `continueImage()` emits one
CHUNK/END/ABORT. Partial records are still completed before switching record type.
No queue depth, credit window, negotiated format, JPEG quality or frame pool grew.

The converter required no algorithm replacement: its scaled libjpeg decode,
integer area resampling, fixed output destination, one quality retry, clean-image
check and caller-owned setjmp contexts remain appropriate. The two input slots
still implement newest-waiting-frame replacement; output leases keep their pool
alive past converter shutdown. Per-image libjpeg allocations remain a documented
exception to preallocation. Sink callbacks must remain nonblocking and return normally; they are not an
exception-isolation boundary and may not invoke lifecycle methods.
Source invalidation discards queued work and checks in-progress work before handoff;
it cannot revoke a lease already handed to a sink.

The recorder captures once and tees compressed MJPEG through a one-buffer leaky
queue to the preview appsink; conversion and tty writes occur on separate workers.
The tap retains buffer lifetime through its callback and unmaps afterward. v0.4
stops recording before preview and stops the display after both image producers.
The map/video exclusion remains page-driven. No display sink, NVENC or DLA was
introduced. Preview uses CPU JPEG work, so real capture latency and thermal load
still require measurement alongside recording on this Orin Nano.

## Validation

Baseline: preview tests and all existing display-link integration scenarios passed.
The new targeted regressions fail against a saved pre-review transport implementation
for all four principal findings. This comparison overlays the old source in a
container; it does not roll back the shared working tree.

Final results: full preview suite, all 23 link integration scenarios, synthetic
recorder Part A (0 failures), ASan/UBSan preview and final transport regressions
with leak detection enabled, and `dashcam_v0_4`/test builds with
`-Wall -Wextra -Werror` passed. Recorder Part B was skipped (no camera).
Further handover details are recorded in HANDOVER.md. Builds/tests use
`l4t-ml-gpio:latest`, isolated `/tmp` or `/build` output directories, and no camera
or display device passthrough. The host reports L4T R36.5.2, consistent with
[NVIDIA's JetPack 6.2.3 announcement](https://forums.developer.nvidia.com/t/jetpack-6-2-3-jetson-linux-36-5-2-is-now-live/379869).

Reproduce the principal checks inside the build container:

```sh
make -j2 BUILD_DIR=/tmp/video-review videopreview_test es3display_test dashcam_v0_4
/tmp/video-review/videopreview_test
/tmp/video-review/es3display_test
# Fast targeted subset, including the deterministic shutdown race:
/tmp/video-review/es3display_test --streaming-regressions
```

The review also runs ASan/UBSan and GCC's analyzer. GCC 11's analyzer reports two
possible-null warnings in standard `std::make_unique` instantiations, treating
throwing `operator new` as returning null. These do not describe a reachable null
return under this build's normal C++ allocation semantics; they are documented,
not suppressed. Ordinary `-Wall -Wextra -Werror` builds are checked separately.

## Remaining priorities

1. Bench-test UGREEN capture → physical ES3C28P presentation at 10–15 fps, including
   reconnects, tab switches and recording load. PTY timings are not USB/TFT timings.
2. Measure CPU, thermal behaviour, capture-to-presented p95 and recorder continuity
   on a sustained drive; tune JPEG quality/FPS only from those measurements.
