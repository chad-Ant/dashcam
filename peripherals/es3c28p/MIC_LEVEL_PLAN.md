# ES3C28P microphone level (SET tab MIC IN indicator)

Plan agreed 2026-11-20. Scope: measure the onboard microphone, drive the SET
tab's MIC IN indicator against the MIC THRESHOLD slider. It is the capture
foundation that [VOICE_COMMAND_PLAN.md](VOICE_COMMAND_PLAN.md) builds on later
(push-to-talk, streaming to the Jetson); nothing here goes over USB.

## Decisions (user, 2026-11-20)

1. **Playback and capture take turns** on I2S0 / the ES8311. Playback always
   wins: a sound stops capture, which resumes after the sound has drained. The
   indicator shows "paused" meanwhile. No full duplex, no shared sample rate.
2. **The microphone runs only while the SET tab is visible** (and nothing is
   playing). Later it is turned on by push-to-talk (voice plan).
3. **Bench readings on the TFT**, not UART (no 3.3 V adapter needed).

## Hardware facts (P0)

Source: LCDWiki schematic `2.8inch_ESP32-S3_Display_Schematic.pdf` (fetched
2026-11-20, sha256 not tracked; the PDF is not in the repo) and Espressif's
ES8311 driver, esp-adf commit `e05f54d46f199df781c2e67c35c0f8ca916a4cdc`
(2026-07-07), `components/esp_codec_dev/device/es8311/es8311.c`
(sha256 `0da6374b…`) and `es8311_reg.h` (`a65d5cdc…`).

- **Microphone:** MIC1, LMA2718B381-OA7, an analog MEMS microphone powered
  from AU_VCC3V3 (100 nF + 10 µF decoupling). It needs no codec bias.
- **Analog path:** MIC_OUT → 0 Ω (L3) → 10 µF (C37) → MIC_P → 1 µF (C31) →
  ES8311 MIC1P. MIC_N → 1 µF (C30) → MIC1N, returned to AU_GND only through
  10 pF/30 pF (C34/C35): a single-ended, pseudo-differential input.
- **Digital path:** ES8311 ASDOUT (net I2S_DO) → **ESP32-S3 GPIO6**. DSDIN
  (I2S_DI) ← GPIO8 (playback), LRCK GPIO7, SCLK GPIO5, MCLK GPIO4, as in
  `es3Board.h`. Codec I2C address 0x18 on the touch bus (SDA16/SCL15).
- **Capture register differences from our playback setup** (`configureCodec()`):
  - `0x0A` SDP out: `0x0C` (16-bit I2S, ADC output **enabled**); playback
    writes `0x4C` (bit 6 = ADC output muted).
  - `0x17` ADC volume: `0xBF` = 0 dB. Never written today; the power-on value
    mutes the ADC.
  - `0x14`: bit 4 selects MIC1P/MIC1N, bits 3..0 the analog PGA gain (the
    driver uses `0x1A`, PGA 30 dB, as playback already writes); bit 6 = digital
    mic, must stay 0.
  - `0x16` ADC gain scale: 0..7 = 0..42 dB in 6 dB steps (`es8311_set_mic_gain`).
  - `0x12`: `0x02` keeps the DAC powered down; `0x32` = `0x00` mutes it; the
    amplifier stays disabled (GPIO1 high).
  - `0x44`: `0x08`. The driver's `0x58` puts the DAC reference in the right
    slot for echo cancellation; not wanted here.
  - Clocks: the existing `0x02..0x08` values equal the driver's row for MCLK
    4.096 MHz at 16 kHz (pre_div 1, mult 1, adc_div 1, ADC OSR 0x10, BCLK/4).
- **Measured in P1 (2026-11-20, `bench/mic_session.py`, guided quiet/talk/clap
  run; raw windows in `~/drive_logs/es3c28p_bench_20261120/mic/session1.*`):**

  | Total gain | Quiet RMS (median / p90) | Speech RMS p90 | Speech peak | Clap peak | Clips |
  |---|---|---|---|---|---|
  | 12 dB (PGA 12) | < −90 dBFS | −79.5 | −62.8 | −51.1 | 0 |
  | 30 dB (PGA 30) | −90.2 / −85.1 | (no speech) | −79.0 | −54.0 | 0 |
  | 42 dB (PGA 30 + scale 12) | −80.2 / −78.0 | −61.6 | −42.1 | −27.6 | 0 |
  | 54 dB (PGA 30 + scale 24) | −66.5 / −61.7 | −33.4 | −17.7 | −15.4 | 0 |

  - **L and R are identical** (L−R difference below 1 LSB in every window):
    the codec sends the ADC on both slots. Use the left slot.
  - DC offset 0 (the codec's ADC high-pass, `0x1B`/`0x1C`, already removes it).
  - The signal is weak: **54 dB is the chosen gain** (PGA 30 dB, ADC scale
    24 dB). It equals Espressif's default `0x16 = 0x24` (bits 2..0 = 4), which
    playback already writes; capture keeps `0x24`. Speech sits ~30 dB above
    the quiet floor; peaks keep 15 dB of headroom.
  - Level mapping starts as **−70..−20 dBFS → 0..100 %**: quiet ≈ 10–15 %,
    speech p90 ≈ 75 %, the default threshold 50 % ≈ −45 dBFS between them.
  - **Screen updates take ~142 ms** (bench text with backgrounds), longer than
    the DMA ring (128 ms): with loop polling the bench dropped ~3 overflows/s;
    with a capture task, 0 over the whole run. The dashboard loop also draws.

## Design

- **Ownership:** `Es3Audio` stays the only owner of the codec and I2S0 and
  gains a capture mode (RX-only channel, 16 kHz, 16-bit Philips stereo slots,
  DIN GPIO6). Codec register sets for playback and capture are separate tables
  applied by the same code. No second class touches the codec.
- **Reading (changed after P1):** a small capture task on core 0 does blocking
  `i2s_channel_read`s and folds the samples into the level meter; the loop
  only reads the published level. The task exists only while capture runs:
  the loop asks it to stop and waits for its acknowledgement before disabling
  and deleting the RX channel (the voice plan's lease rule). The I2S
  receive-overflow callback counts drops.
- **Level maths:** portable `lib/es3Audio/MicLevel.h`, host-tested: select the
  verified slot, first-order DC-removing high-pass, RMS over 50 ms, dBFS,
  map −60..0 dBFS to 0..100 % (range tuned in P1), fast attack / slow release,
  discard the first ~100 ms after capture starts.
- **Detection:** green at level ≥ threshold; back to grey after 300 ms at
  least 5 points below it. A new `kMicPaused` state shows "paused" during
  sounds, next to the existing "no source".
- **Coordination (`main.cpp`):** capture wanted = SET tab visible
  (`Es3Graphics::micMeterWanted()`) and not playing; `Es3Audio` starts/stops
  capture; the level goes to `Es3Graphics::setMicLevel()`. Wire protocol and
  STATUS unchanged.

## Phases

| | Work | Gate |
|---|---|---|
| P0 | Hardware facts (this file). | Done 2026-11-20. |
| P1 | `bench/mic_capture/` + `bench/mic_session.py` (guided run, prompts on the TFT, results over USB). | Done 2026-11-20: left slot, 54 dB, −70..−20 dBFS, capture task. |
| P2 | `MicLevel.h` + `MicDetector` (SettingsLayout.h), `tests/mic_level_test.cpp`. | Done 2026-11-20: all suites, C++11/17, sanitizers. |
| P3 | `Es3Audio` capture mode (capture task, stop handshake, one codec table for both modes). | Done 2026-11-20: build clean, flashed. |
| P4 | Graphics + `main.cpp` wiring, "paused"/"no input" states, live level bar under the threshold slider. | Done 2026-11-20. |
| P5 | Board acceptance; README, HANDOVER, commit. | Done 2026-11-20, all passed (user): quiet → grey; speech → green, grey ~⅓ s after; threshold knob moves the switch point; TEST → paused then resumes; touch and sliders responsive. |

## Not in scope

Voice activity detection, pre-roll, PCM streaming, push-to-talk UI and any USB
protocol change (all in the voice plan). The threshold only drives the
indicator.
